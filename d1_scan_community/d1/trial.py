from clearance import BodyClearance, BodyBlocked
from map_route import path_from_map
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import os
import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from rclpy.time import Time
from scan_planner_msgs.msg import Bspline
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def sdk_lateral_speed(requested, previous):
    # Match the D1 SDK's nonzero side-step range before the bridge clamps it.
    threshold = .015 if previous * requested > 0. else .030
    if abs(requested) < threshold:
        return 0.
    return math.copysign(min(.12, max(.10, abs(requested))), requested)


def main():
    started = float(os.getenv("D1_TASK_MONOTONIC", time.monotonic()))
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--yaw', type=float, required=True)
    parser.add_argument('--shadow', action='store_true')
    parser.add_argument('--shadow-arm-delay',type=float,default=0.)
    options = parser.parse_args()
    if options.shadow_arm_delay and not options.shadow:
        parser.error('delay injection requires shadow mode')
    if not options.goal and not options.shadow:
        parser.error('a site-selected goal is required')
    rclpy.init()
    node = rclpy.create_node('d1_planning_preview')
    clearance = BodyClearance(node)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(RosPath, '/d1_community/initial_path',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    output = node.create_publisher(Twist, '/navigation/approved_velocity' if not options.shadow else '/d1_community/shadow_velocity', 1)
    arm = node.create_client(SetBool, '/d1/arm_motion')
    commands = []
    losses = []
    node.create_subscription(Bool, '/d1/motion_armed', lambda m: losses.append(True) if armed and not options.shadow and not m.data else None, 1)
    last_candidate = [0.]
    def candidate(msg):
        last_candidate[0] = time.monotonic()
        commands[:] = [(time.monotonic(),msg)]
    node.create_subscription(Twist, '/d1_community/candidate', candidate,
                             QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    ready = []
    armed = False
    import select
    planner_exit = select.poll()
    planner_pid = os.environ.get('D1_SCAN_PID')
    if planner_pid:
        planner_fd = os.pidfd_open(int(planner_pid))
        planner_exit.register(planner_fd, select.POLLIN)
    parent_pid = int(os.getenv("D1_TASK_PARENT", "0"))

    def check_planner():
        if parent_pid and not os.path.exists(f"/proc/{parent_pid}"):
            raise RuntimeError("网页控制服务已退出")
        if planner_exit.poll(0):
            raise RuntimeError('SCAN 局部规划进程已退出，任务已停止')
    node.create_subscription(Bool, '/localization/ready', lambda msg: ready.append(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    latest = []
    trajectory_received = [0.]
    moving = [False]
    def collect_trajectory(message):
        commands.clear()
        trajectory_received[0] = time.monotonic()
        latest[:] = [message]
        moving[0] = bool(message.pos_pts) and any(
            math.hypot(point.x-message.pos_pts[0].x, point.y-message.pos_pts[0].y) > 1.e-6
            for point in message.pos_pts)
    node.create_subscription(Bspline, '/d1_community/bspline', collect_trajectory, 10)
    def submit_route():
        until = time.monotonic() + 12
        while time.monotonic() < until:
            check_planner()
            rclpy.spin_once(node, timeout_sec=.15)
            if not clearance.ready() or not ready or not ready[-1] or not publisher.get_subscription_count():
                continue
            try:
                anchored = buffer.lookup_transform('map', 'base_link', Time())
                map_to_odom = buffer.lookup_transform('odom', 'map', Time())
                odom_base = buffer.lookup_transform('odom', 'base_link', Time())
            except TransformException:
                continue
            if not 0 <= node.get_clock().now().nanoseconds - Time.from_msg(anchored.header.stamp).nanoseconds < 2e9:
                continue
            break
        else:
            raise RuntimeError('等待有效定位、障碍点云或规划跟踪节点超时')
        print(f"点云、局部地图和位姿就绪：启动后 {time.monotonic()-started:.2f} 秒", flush=True)
        start = anchored.transform.translation
        heading = anchored.transform.rotation
        direction = math.atan2(2 * (heading.w * heading.z + heading.x * heading.y),
                               1 - 2 * (heading.y**2 + heading.z**2))
        translation, rotation = map_to_odom.transform.translation, map_to_odom.transform.rotation
        convert = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
        route = path_from_map(np.array([start.x, start.y]), direction, options.goal)
        positions = convert.apply(np.column_stack((route, np.full(len(route), start.z))))
        positions += [translation.x, translation.y, translation.z]
        positions[:, 2] = odom_base.transform.translation.z
        message = RosPath()
        message.header.frame_id = 'odom'
        message.header.stamp = node.get_clock().now().to_msg()
        for x, y, z in positions[1:]:  # Upstream prepends the measured start pose.
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z - .4)
            pose.pose.orientation.w = 1.
            message.poses.append(pose)
        route_length = float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())
        print(json.dumps({'start': route[0].tolist(), 'goal': route[-1].tolist(), 'map_route': route.tolist(),
                          'length_m': route_length}), flush=True)
        endpoint = message.poses[-1]
        yaw = options.yaw + convert.as_euler('xyz')[2]
        endpoint.pose.orientation.w = math.cos(yaw/2)
        endpoint.pose.orientation.z = math.sin(yaw/2)
        publisher.publish(message)
        return positions[-1,:2], yaw, message.header.stamp

    try:
        goal_xy, yaw, stamp = submit_route()
        first_stamp = Time.from_msg(stamp).nanoseconds
        deadline = time.monotonic() + 180.
        last_reason = None
        logged_at = 0.
        lateral = 0.
        settled = None
        first_velocity = False
        checked = None
        checked_at = 0.
        gap = 0.
        while time.monotonic() < deadline:
            check_planner()
            rclpy.spin_once(node, timeout_sec=.015)
            if losses or not ready[-1] or not clearance.ready():
                raise RuntimeError('定位、障碍观测或运动授权失效')
            now = time.monotonic()
            pose_stamp, position, rotation = clearance.pose
            ros_now = node.get_clock().now().nanoseconds
            if not 0 <= ros_now - pose_stamp < 350_000_000:
                raise RuntimeError('机身里程计过期')
            cmd = Twist()
            reason = '等待分支的有效轨迹和跟踪速度'
            at_goal = np.linalg.norm(position[:2] - goal_xy) <= .15
            if not at_goal:
                settled = None
            if at_goal:
                current = rotation.as_euler('xyz')[2]
                error = math.atan2(math.sin(yaw-current), math.cos(yaw-current))
                if abs(error) <= math.radians(5):
                    settled = now if settled is None else settled
                    if now - settled >= .45:
                        print('ARRIVED', flush=True)
                        return
                else:
                    settled = None
                    nearby = clearance.observed_obstacles()
                    if len(nearby) and np.linalg.norm(nearby[:,:2]-position[:2],axis=1).min() <= .48:
                        reason = '终点转向净空不足'
                    else:
                        cmd.angular.z = float(np.clip(error,-.25,.25))
            elif commands and latest:
                received, candidate_cmd = commands.pop()
                spline = latest[0]
                start = Time.from_msg(spline.start_time).nanoseconds
                if (received >= trajectory_received[0] and now-received < .15
                        and start >= first_stamp and spline.order > 0
                        and len(spline.pos_pts) > spline.order
                        and len(spline.knots) == len(spline.pos_pts)+1+spline.order
                        and moving[0]):
                    age = (ros_now-start)*1.e-9
                    duration = spline.knots[-1-spline.order]-spline.knots[spline.order]
                    if -.05 <= age < duration:
                        if checked is not spline or now-checked_at >= .10:
                            gap = clearance.path_clearance(spline,age)
                            checked, checked_at = spline, now
                        if gap is not None and gap > .48:
                            cmd = candidate_cmd
                            reason = '分支跟踪器输出零速度'
                        else:
                            reason = f'D1 近期轨迹净空 {gap} m'
            elif not commands and now-last_candidate[0] < .12 and latest and moving[0]:
                # No command callback in this spin. The bridge keeps its own short watchdog.
                continue
            if not np.isfinite([cmd.linear.x,cmd.linear.y,cmd.angular.z]).all():
                raise RuntimeError('控制器输出非有限速度')
            cmd.linear.x = float(np.clip(cmd.linear.x,-.12,.12))
            cmd.linear.y = sdk_lateral_speed(cmd.linear.y,lateral)
            cmd.angular.z = float(np.clip(cmd.angular.z,-.25,.25))
            if abs(cmd.linear.x) < .05:
                cmd.linear.x = 0.
            if abs(cmd.angular.z) < .02:
                cmd.angular.z = 0.
            try:
                clearance.check(cmd)
            except BodyBlocked:
                cmd = Twist()
                reason = 'D1 机身刹停保护'
            active = any((cmd.linear.x,cmd.linear.y,cmd.angular.z))
            reason = '运动' if active else reason
            if reason.startswith('D1 近期'):
                reason = f'D1 近期轨迹净空不足（{gap:.2f} m）' if gap is not None else '局部地图未就绪'
            if reason != last_reason and now-logged_at > .25:
                print(f'试验 {now-started:.2f}s {reason}',flush=True)
                last_reason, logged_at = reason, now
            if not armed and active:
                if not options.shadow:
                    fut=arm.call_async(SetBool.Request(data=True))
                    rclpy.spin_until_future_complete(node,fut,timeout_sec=.5)
                    if not fut.done() or not fut.result().success:
                        raise RuntimeError('D1 控制桥未授权')
                armed=True
                commands.clear()
                print('TRACKING',flush=True)
                continue
            if armed:
                output.publish(cmd)
            lateral = cmd.linear.y
            if active and not first_velocity:
                first_velocity=True
                print('首条非零指令',flush=True)
            if options.shadow and first_velocity:
                print('SHADOW_OK',flush=True)
                return
        raise RuntimeError('试验时间已到，已停车')
    finally:
        if armed and not options.shadow:
            output.publish(Twist())
            future=arm.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(node,future,timeout_sec=.4)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
