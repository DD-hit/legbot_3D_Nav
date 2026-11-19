import json
import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from std_msgs.msg import Bool, String


def coordinates(pose):
    point = pose.pose.position
    return [point.x, point.y, point.z]


def heading(pose):
    rotation = pose.pose.orientation
    norm = math.hypot(rotation.x, rotation.y, rotation.z, rotation.w)
    if norm == 0:
        return None
    if not math.isfinite(norm) or abs(norm - 1) > 0.01:
        raise ValueError('目标朝向四元数无效')
    return math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                      norm * norm - 2 * (rotation.y**2 + rotation.z**2))


class WaypointTasks:
    def __init__(self, mission, latched):
        self.mission = mission
        self.handle = None
        self.result = None
        self.state = dict(id='', phase='idle', index=0, points=[], reason='')
        self.route_pub = mission.create_publisher(Path, '/pct/route_request', 10)
        self.status_pub = mission.create_publisher(String, '/navigation/task_status', latched)
        self.leg_pub = mission.create_publisher(Bool, '/scan/leg_complete', latched)
        self.leg_pub.publish(Bool(data=False))
        mission.create_subscription(PoseStamped, '/scan/terminal_turn_clear', self.on_turn_clear, 10)
        self.server = ActionServer(mission, FollowWaypoints, '/navigation/follow_waypoints',
                                   self.execute, goal_callback=self.validate,
                                   handle_accepted_callback=self.accept,
                                   cancel_callback=lambda handle: CancelResponse.ACCEPT)
        self.publish()

    @property
    def active(self):
        return self.handle is not None

    def publish(self):
        self.status_pub.publish(String(data=json.dumps(self.state, ensure_ascii=False)))

    def validate(self, goal):
        mission = self.mission
        odom_age = mission.get_clock().now().nanoseconds - getattr(mission, 'odom_stamp_ns', 0)
        if (self.active or mission.fix_stamp_ns is None or not mission.ready or not 0 <= odom_age <= 2_000_000_000
                or (mission.web and (mission.web.seed_pending is not None or mission.web.outside_fix == mission.fix_stamp_ns))
                or not 1 <= len(goal.poses) <= 100):
            return GoalResponse.REJECT
        for pose in goal.poses:
            if pose.header.frame_id != 'map' or not all(map(math.isfinite, coordinates(pose))):
                return GoalResponse.REJECT
            try:
                heading(pose)
            except ValueError:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def accept(self, handle):
        self.handle = handle
        self.result = Future()
        self.replanned = False
        self.state = dict(id=bytes(handle.goal_id.uuid).hex(), phase='planning', index=0,
                          points=[coordinates(p) for p in handle.request.poses],
                          headings=[heading(p) for p in handle.request.poses], reason='')
        self.mission.complete_pub.publish(Bool(data=False))
        self.begin_leg()
        handle.execute()

    async def execute(self, handle):
        return await self.result

    def begin_leg(self):
        mission = self.mission
        mission.raw_path = mission.request_stamp = mission.goal_stamp = None
        mission.plan_accepted = mission.completed = mission.started = False
        mission.publish_permit()
        mission.plan_ready_pub.publish(Bool(data=False))
        self.leg_pub.publish(Bool(data=False))
        self.state.update(phase='planning', reason='已从停车位置重新规划本段' if self.replanned else '')
        self.progress_time = time.monotonic()
        self.progress_pose = mission.initial_odom
        self.handle.publish_feedback(FollowWaypoints.Feedback(current_waypoint=self.state['index']))
        self.publish()

    def can_request(self):
        return self.active and self.state['phase'] == 'planning' and self.route_pub.get_subscription_count() > 0

    def request_route(self, start):
        goal = self.handle.request.poses[self.state['index']]
        self.route_pub.publish(Path(header=start.header, poses=[start, goal]))

    def on_turn_clear(self, msg):
        mission = self.mission
        if (self.active and self.state['phase'] == 'planning' and mission.started and not mission.plan_accepted
                and self.state['headings'][self.state['index']] is not None and mission.fix_stamp_ns is not None
                and (msg.header.stamp.sec, msg.header.stamp.nanosec) == mission.goal_stamp):
            mission.plan_accepted = True
            mission.publish_permit()
            mission.plan_ready_pub.publish(Bool(data=True))

    def reached(self):
        if not self.active or self.state['phase'] not in ('planning', 'walking'):
            return
        if self.handle.is_cancel_requested:
            self.stop('canceled', '用户取消任务')
            return
        self.leg_pub.publish(Bool(data=True))
        self.state['index'] += 1
        if self.state['index'] == len(self.state['points']):
            self.mission.complete_pub.publish(Bool(data=True))
            self.finish('succeeded')
        else:
            self.state['phase'] = 'at_waypoint'
            self.replanned = False
            self.arrival_stamp = self.mission.odom_stamp_ns
            self.publish()

    def stop(self, terminal, reason):
        if not self.active:
            return
        if self.state['phase'] == 'stopping':
            if self.stop_terminal == 'replan' and terminal != 'replan':
                self.stop_terminal = terminal
                self.state['reason'] = reason
                self.publish()
            return
        self.state.update(phase='stopping', reason=reason)
        self.stop_terminal = terminal
        self.stop_anchor = None
        self.leg_pub.publish(Bool(data=True))
        self.mission.cancel_localized_mission()
        self.publish()

    def finish(self, phase):
        index = self.state['index']
        result = FollowWaypoints.Result(missed_waypoints=list(range(index, len(self.state['points']))))
        self.state['phase'] = phase
        if phase == 'succeeded':
            self.state['reason'] = ''
        if self.handle.is_cancel_requested:
            self.handle.canceled()
        elif phase == 'succeeded':
            self.handle.succeed()
        else:
            self.handle.abort()
        self.result.set_result(result)
        self.handle = None
        self.publish()

    def tick(self):
        if not self.active:
            return
        if self.handle.is_cancel_requested:
            self.stop('canceled', '用户取消任务')
        mission = self.mission
        phase = self.state['phase']
        if phase == 'stopping':
            pose = mission.initial_odom
            stamp = getattr(mission, 'odom_stamp_ns', 0)
            if pose is None or not 0 <= mission.get_clock().now().nanoseconds - stamp <= 500_000_000:
                self.stop_anchor = None
                return
            if self.stop_anchor:
                previous, first = self.stop_anchor
                angle = math.remainder(pose[3] - previous[3], 2 * math.pi)
                if stamp >= first and math.dist(pose[:2], previous[:2]) <= 0.025 and abs(angle) <= 0.05:
                    if stamp - first >= 600_000_000:
                        if self.stop_terminal == 'replan':
                            self.replanned = True
                            self.begin_leg()
                        else:
                            self.finish(self.stop_terminal)
                    return
            self.stop_anchor = (pose, stamp)
        elif phase == 'at_waypoint':
            if mission.odom_stamp_ns - self.arrival_stamp >= 700_000_000:
                self.begin_leg()
        else:
            if mission.plan_accepted and phase == 'planning':
                self.state['phase'] = 'walking'
                self.publish()
            if mission.plan_accepted and mission.initial_odom and self.progress_pose:
                if (math.dist(mission.initial_odom[:2], self.progress_pose[:2]) > 0.06 or
                        abs(math.remainder(mission.initial_odom[3] - self.progress_pose[3], 2 * math.pi)) > 0.09):
                    self.progress_time = time.monotonic()
                    self.progress_pose = mission.initial_odom
            stalled = time.monotonic() - self.progress_time
            if stalled > 6 and mission.started and not self.replanned:
                self.stop('replan', '当前段持续无进展，站稳后从当前位置重新规划一次')
            elif stalled > (20 if self.replanned and mission.started else 90):
                self.stop('failed', '等待规划或行走进展超时')
            elif mission.started and stalled > 2:
                reason = '暂未检测到位姿进展，等待可执行轨迹'
                if self.state['reason'] != reason:
                    self.state['reason'] = reason
                    self.publish()
            elif mission.plan_accepted and self.state['reason']:
                self.state['reason'] = ''
                self.publish()
