import argparse
import json
import math
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Event, Thread

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
import speed_settings
from manual_drive import SERVICE
from single_task import SingleTask
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from floor_map import FloorMap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--port', type=int, default=8871)
    options = parser.parse_args()
    manifest = options.manifest.resolve()
    profile = yaml.safe_load(manifest.read_text())
    atlas = FloorMap(str(manifest.parent / (profile['tomogram'] + '.pickle')),
                     profile['floors'], profile['id'], profile['version'])
    page = Path(__file__).with_name('localization.html').read_bytes()
    rclpy.init()
    node = rclpy.create_node('d1_localization_web')
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    controls = {action: node.create_client(Trigger, '/d1/' + action) for action in ('stand', 'lie', 'stop', 'forward', 'backward', 'left', 'right', 'turn_left', 'turn_right')}
    manual = node.create_client(SetParametersAtomically, SERVICE)
    assets = Path(__file__).with_name('workbench')
    display = (manifest.parent / profile['display']).read_text().replace('<', r'\u003c')
    navigation = (assets / 'index.html').read_text().replace('@FLOOR_DISPLAY@', display).encode()
    status = {'state': 'WAITING_FOR_INITIAL_POSE'}
    cancel_motion = node.create_publisher(Empty, "/d1/cancel_motion", 10)
    tasks = SingleTask()
    latest_cloud = None
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    submitted = 0

    def on_status(message):
        nonlocal status
        status = json.loads(message.data)

    def on_cloud(message):
        nonlocal latest_cloud
        latest_cloud = Time.from_msg(message.header.stamp)

    node.create_subscription(String, '/localization/status', on_status,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(PointCloud2, '/fast_lio/cloud_registered', on_cloud, qos_profile_sensor_data)

    class Handler(BaseHTTPRequestHandler):
        def send(self, data, content_type='application/json', code=200):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/api/speeds':
                return self.send(json.dumps(speed_settings.load()).encode())
            if self.path == '/diagnostics':
                return self.send(page, 'text/html; charset=utf-8')
            if self.path == '/navigation':
                return self.send(navigation, 'text/html; charset=utf-8')
            files = {'/': ('home.html','text/html'), '/manual': ('manual.html','text/html'),
                     '/manual.js': ('manual.js','text/javascript'), '/console.css': ('console.css','text/css'),
                     '/console.js': ('console.js','text/javascript')}
            if self.path in files:
                name, mime = files[self.path]
                return self.send((assets / name).read_bytes(), mime + '; charset=utf-8')
            if self.path in ('/map','/api/map'):
                return self.send(atlas.payload)
            if self.path.startswith('/api/events?'):
                self.send_response(200)
                self.send_header('Content-Type','text/event-stream')
                self.end_headers()
                try:
                    while rclpy.ok():
                        result = self.live_status()
                        task,route = tasks.snapshot()
                        snapshot = dict(localization=result, pose=result.get('pose'), standing=result['cloud'],
                                        ready=result['control'] and result['cloud'] and 'pose' in result, task=task, route=route, dependency_error='')
                        self.wfile.write(b'data: ' + json.dumps(snapshot).encode() + b'\n\n')
                        self.wfile.flush()
                        time.sleep(.3)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            if self.path == '/status':
                return self.send(json.dumps(self.live_status()).encode())
            self.send_error(404)

        def live_status(self):
            result = status if status.get('seed_stamp_ns', 0) and status['seed_stamp_ns'] >= submitted else (
                {'state': 'COLLECTING'} if submitted else status)
            result = dict(result, cloud=latest_cloud is not None and
                          0 <= node.get_clock().now().nanoseconds - latest_cloud.nanoseconds < 2e9)
            result['control'] = all(client.service_is_ready() for client in controls.values())
            if result['state'] == 'LOCALIZED':
                try:
                    transform = buffer.lookup_transform(profile['frame'], 'base_link', Time())
                    when = Time.from_msg(transform.header.stamp).nanoseconds
                    if result['seed_stamp_ns'] <= when and 0 <= node.get_clock().now().nanoseconds - when < 2e9:
                        xyz, quat = transform.transform.translation, transform.transform.rotation
                        result['pose'] = [xyz.x, xyz.y, xyz.z, math.atan2(
                            2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y**2 + quat.z**2))]
                except TransformException:
                    pass
            return result

        def do_POST(self):
            nonlocal submitted
            host = self.headers.get('Host', '')
            if (host.partition(':')[0] not in ('127.0.0.1', 'localhost')
                    or self.headers.get('Origin') != 'http://' + host):
                return self.send_error(403)
            if self.path == '/api/speeds':
                try:
                    size = int(self.headers.get('Content-Length', 0))
                    if not 0 < size <= 512:
                        raise ValueError('请求长度无效')
                    settings = speed_settings.validate(json.loads(self.rfile.read(size)))
                    with tasks.lock:
                        if tasks.process is not None and settings['navigation'] != speed_settings.load()['navigation']:
                            raise ValueError('请在当前导航结束后修改导航速度')
                        speed_settings.save(settings)
                    return self.send(json.dumps(settings).encode())
                except (ValueError, OSError) as error:
                    return self.send(json.dumps({'error': str(error)}).encode(), code=409)
            if self.path == '/api/manual':
                if not manual.service_is_ready():
                    return self.send(json.dumps({'error':'手动控制未连接，请启用 D1_WEB_CONTROL 并检查 SDK'}).encode(),code=503)
                try:
                    size = int(self.headers.get('Content-Length', 0))
                    if not 0 < size <= 512:
                        raise ValueError()
                    body = json.loads(self.rfile.read(size))
                    if not isinstance(body,dict) or set(body)-{'action','owner','seq','token','x','y','turn'}:
                        raise ValueError()
                    req = SetParametersAtomically.Request()
                    req.parameters = [Parameter(k,value=v).to_parameter_msg() for k,v in body.items()]
                except (TypeError,ValueError,OverflowError,AssertionError):
                    return self.send(b'{"error":"invalid command"}',code=400)
                done = Event()
                future = manual.call_async(req)
                future.add_done_callback(lambda _: done.set())
                if not done.wait(.7):
                    return self.send(b'{"error":"control timeout"}',code=504)
                result = future.result().result
                key = 'message' if result.successful else 'error'
                return self.send(json.dumps({key:result.reason}).encode(),code=200 if result.successful else 409)
            if self.path in ('/api/tasks','/api/cancel'):
                try:
                    size = int(self.headers.get('Content-Length',0))
                    if not 0 < size <= 4096:
                        raise ValueError('请求长度无效')
                    body = json.loads(self.rfile.read(size))
                    if self.path == '/api/cancel':
                        if tasks.cancel(body['id']):
                            cancel_motion.publish(Empty())
                    else:
                        live=self.live_status()
                        if not live['control'] or not live['cloud'] or live['state']!='LOCALIZED' or 'pose' not in live:
                            raise ValueError('请检查定位和控制连接')
                        tasks.start(body,atlas)
                except (ValueError,TypeError,KeyError,IndexError) as error:
                    return self.send(json.dumps({'error':str(error)}).encode(),code=409)
                return self.send(b'{"accepted":true}')
            if self.path.startswith('/control/'):
                action = self.path.removeprefix('/control/')
                client = controls.get(action) if action == 'stop' else None
                if client is None or not client.service_is_ready():
                    return self.send(json.dumps({'message': '控制服务未启用'}).encode(), code=503)
                event = Event()
                future = client.call_async(Trigger.Request())
                future.add_done_callback(lambda _: event.set())
                if not event.wait(3):
                    return self.send(json.dumps({'message': '控制响应超时，请现场确认状态'}).encode(), code=504)
                response = future.result()
                return self.send(json.dumps({'ok': response.success, 'message': response.message}).encode(),
                                 code=200 if response.success else 409)
            if self.path not in ('/seed','/api/seed'):
                return self.send_error(404)
            try:
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 256:
                    raise ValueError()
                body = json.loads(self.rfile.read(size))
                if self.path == '/api/seed':
                    body = [*atlas.point(body,False)[:2], body['yaw']]
                x, y, yaw = map(float,body)
                if not all(math.isfinite(value) for value in (x, y, yaw)) or not atlas.contains_base((x, y)):
                    raise ValueError()
            except (TypeError, ValueError, KeyError):
                return self.send(b'{}', code=400)
            message = PoseWithCovarianceStamped()
            now = node.get_clock().now()
            message.header.stamp, message.header.frame_id = now.to_msg(), profile['frame']
            pose = message.pose.pose
            pose.position.x, pose.position.y, pose.position.z = x, y, float(profile['floors'][0][0])
            pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            submitted = now.nanoseconds
            publisher.publish(message)
            self.send(b'{}', code=202)

    server = ThreadingHTTPServer(('127.0.0.1', options.port), Handler)
    Thread(target=server.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tasks.cancel()
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
