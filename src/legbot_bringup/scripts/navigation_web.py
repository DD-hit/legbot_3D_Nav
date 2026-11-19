import json
import math
import queue
import threading
import time
import uuid
from concurrent.futures import Future, TimeoutError
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from tf2_ros import TransformException
from unique_identifier_msgs.msg import UUID

from floor_map import FloorMap
from waypoint_tasks import coordinates


class NavigationWeb:
    def __init__(self, mission, port):
        self.mission = mission
        self.atlas = FloorMap()
        self.localization = {'state': 'WAITING_FOR_INITIAL_POSE'}
        self.seed = None
        self.seed_pending = None
        self.expired_seed = None
        self.display_seed_stamp = None
        self.outside_fix = None
        self.pending = queue.Queue(maxsize=32)
        self.condition = threading.Condition()
        self.snapshot = b'{}'
        self.online = True
        self.seed_pub = mission.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        action = '/navigation/follow_waypoints'
        self.client = ActionClient(mission, FollowWaypoints, action)
        self.cancel = mission.create_client(CancelGoal, action + '/_action/cancel_goal')
        self.root = Path(get_package_share_directory('legbot_bringup')) / 'web'
        self.server = ThreadingHTTPServer(('127.0.0.1', port), self.handler())
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.timer = mission.create_timer(0.2, self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))
        mission.get_logger().info(f'Navigation console listening on http://127.0.0.1:{port}')

    def close(self):
        with self.condition:
            self.online = False
            self.condition.notify_all()
        self.server.shutdown()
        self.server.server_close()

    def update(self):
        while not self.pending.empty():
            path, body, reply = self.pending.get_nowait()
            if not reply.set_running_or_notify_cancel():
                continue
            try:
                self.command(path, body, reply)
            except (KeyError, IndexError, TypeError, ValueError, RuntimeError, OverflowError) as error:
                reply.set_exception(ValueError(str(error)))
        mission = self.mission
        pose = None
        if self.seed_pending is not None and time.monotonic() >= self.seed_deadline:
            self.expired_seed = self.seed_pending
            self.seed_pending = None
            self.localization = {'state': 'FAILED', 'reason': '定位节点未及时返回本次结果'}
        if self.localization.get('state') == 'LOCALIZED' and mission.fix_stamp_ns is not None:
            try:
                tf = mission.buffer.lookup_transform('map', 'base', Time())
                stamp = Time.from_msg(tf.header.stamp).nanoseconds
                if stamp >= mission.fix_stamp_ns and 0 <= mission.get_clock().now().nanoseconds - stamp < 2_000_000_000:
                    pos, rot = tf.transform.translation, tf.transform.rotation
                    yaw = math.atan2(2*(rot.w*rot.z + rot.x*rot.y), 1-2*(rot.y**2 + rot.z**2))
                    pose = [pos.x, pos.y, pos.z, yaw]
            except TransformException:
                pass
        if pose is not None and not self.atlas.contains_base(pose[:2]):
            self.outside_fix = mission.fix_stamp_ns
            mission.tasks.stop('failed', '机身估计已超出地图，请重新定位')
        outside = mission.fix_stamp_ns is not None and self.outside_fix == mission.fix_stamp_ns
        ready = mission.ready and pose is not None and not outside and self.seed_pending is None
        route = [coordinates(p) for p in mission.raw_path.poses] if mission.raw_path and mission.tasks.active else []
        snapshot = dict(localization={k: self.localization[k] for k in ('state', 'reason', 'pose', 'request_id', 'seed_stamp_ns') if k in self.localization},
                        standing=mission.ready, ready=ready, seed=self.seed, pose=pose, task=mission.tasks.state, route=route)
        if outside:
            snapshot.update(localization={'state': 'LOST', 'reason': '机身估计已超出地图，请重新定位'}, pose=None)
        data = json.dumps(snapshot, ensure_ascii=False, allow_nan=False).encode()
        with self.condition:
            if data != self.snapshot:
                self.snapshot = data
                self.condition.notify_all()

    def command(self, path, body, reply):
        mission = self.mission
        if not isinstance(body, dict):
            raise ValueError('请求格式无效')
        if path == '/api/seed':
            if self.seed_pending is not None:
                raise ValueError('正在处理本次初始定位，请等待匹配结果')
            if mission.tasks.active or self.seed_pub.get_subscription_count() == 0 or not mission.ready:
                raise ValueError('请等待机器狗就绪并结束当前任务')
            xyz = self.atlas.point(body, False)
            yaw = float(body['yaw'])
            if not math.isfinite(yaw):
                raise ValueError('朝向无效')
            message = PoseWithCovarianceStamped()
            message.header.frame_id = 'map'
            message.header.stamp = mission.get_clock().now().to_msg()
            pose = message.pose.pose
            pose.position.x, pose.position.y, pose.position.z = xyz
            pose.orientation.z, pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.seed = [*xyz, yaw]
            self.seed_pending = Time.from_msg(message.header.stamp).nanoseconds
            self.display_seed_stamp = self.seed_pending
            self.seed_deadline = time.monotonic() + 35
            self.localization = {'state': 'COLLECTING', 'reason': '等待本次点云'}
            mission.cancel_localized_mission()
            mission.fix_stamp_ns = None
            self.seed_pub.publish(message)
            reply.set_result({'accepted': True})
            return
        if not isinstance(body.get('id'), str):
            raise ValueError('任务编号无效')
        goal_id = UUID(uuid=list(uuid.UUID(body['id']).bytes))
        if path == '/api/cancel':
            if not mission.tasks.active or bytes(goal_id.uuid).hex() != mission.tasks.state['id']:
                raise ValueError('任务已结束或已切换')
            request = CancelGoal.Request()
            request.goal_info.goal_id = goal_id
            future = self.cancel.call_async(request)
            check = lambda response: bool(response.goals_canceling)
        else:
            if not self.client.server_is_ready() or self.seed_pending is not None or mission.fix_stamp_ns is None:
                raise ValueError('请先完成定位')
            items = body['points']
            if not isinstance(items, list) or not 1 <= len(items) <= 100:
                raise ValueError('每个任务支持 1 至 100 个点')
            goal = FollowWaypoints.Goal()
            for item in items:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.atlas.point(item, True)
                if not isinstance(item.get('yaw'), (int, float)) or isinstance(item['yaw'], bool) or not math.isfinite(item['yaw']):
                    raise ValueError('请拖动指定每个目标的位置和朝向')
                yaw = item['yaw']
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                goal.poses.append(pose)
            future = self.client.send_goal_async(goal, goal_uuid=goal_id)
            check = lambda response: response.accepted

        def answered(response):
            try:
                if not check(response.result()):
                    raise ValueError('命令被拒绝，请查看当前任务和定位状态')
                reply.set_result({'accepted': True})
            except Exception as error:
                reply.set_exception(ValueError(str(error)))
        future.add_done_callback(answered)

    def handler(self):
        app = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                pass

            def send(self, code, mime, data):
                self.send_response(code)
                self.send_header('Content-Type', mime)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def trusted(self):
                host = self.headers.get('Host', '')
                return (host.split(':')[0] in ('localhost', '127.0.0.1') and
                        self.headers.get('Origin', 'http://' + host) == 'http://' + host)

            def do_GET(self):
                if not self.trusted():
                    return self.send_error(403)
                path = urlsplit(self.path).path
                if path == '/api/map':
                    return self.send(200, 'application/json', app.atlas.payload)
                if path == '/api/state':
                    return self.send(200, 'application/json', app.snapshot)
                if path == '/api/events':
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    previous = None
                    try:
                        while app.online:
                            with app.condition:
                                app.condition.wait_for(lambda: previous != app.snapshot or not app.online, timeout=3)
                                current = app.snapshot
                            self.wfile.write(b'data: ' + current + b'\n\n' if current != previous else b': ping\n\n')
                            self.wfile.flush()
                            previous = current
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    return
                files = {'/': ('index.html', 'text/html; charset=utf-8'), '/console.js': ('console.js', 'text/javascript'),
                         '/console.css': ('console.css', 'text/css')}
                if path not in files:
                    return self.send_error(404)
                name, mime = files[path]
                self.send(200, mime, (app.root / name).read_bytes())

            def do_POST(self):
                if not self.trusted() or self.headers.get_content_type() != 'application/json':
                    return self.send_error(403)
                if self.path not in ('/api/seed', '/api/tasks', '/api/cancel'):
                    return self.send_error(404)
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 32768:
                        raise ValueError('请求长度无效')
                    body = json.loads(self.rfile.read(length))
                    reply = Future()
                    app.pending.put_nowait((self.path, body, reply))
                    data = reply.result(timeout=8)
                    self.send(200, 'application/json', json.dumps(data).encode())
                except (ValueError, UnicodeError) as error:
                    self.send(409, 'application/json', json.dumps({'error': str(error)}).encode())
                except (TimeoutError, queue.Full):
                    reply.cancel()
                    self.send(503, 'application/json', b'{"error":"Command timed out"}')
        return Handler
