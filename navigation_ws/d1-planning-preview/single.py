from clearance import BodyClearance, BodyBlocked
from preview import path_from_map, TemporarilyBlocked
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import os
import argparse
import json
import math
from pathlib import Path
import pickle
import time

import a_star
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from rclpy.time import Time
from scan_planner.msg import Bspline
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
    publisher = node.create_publisher(RosPath, '/d1_trial/initial_path',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    goal_pub = node.create_publisher(PoseStamped, '/d1_trial/tracker_goal', 10)
    output = node.create_publisher(Twist, '/navigation/approved_velocity' if not options.shadow else '/d1_trial/shadow_velocity', 1)
    arm = node.create_client(SetBool, '/d1/arm_motion')
    commands, acknowledgements = [], []
    losses = []
    node.create_subscription(Bool, '/d1/motion_armed', lambda m: losses.append(True) if armed and not options.shadow and not m.data else None, 1)
    def candidate(msg):
        commands[:] = [(time.monotonic(),msg)]
    node.create_subscription(Twist, '/d1_trial/candidate', candidate,
                             QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(PoseStamped, '/scan/goal_stop_ack', lambda m: acknowledgements.append(m.header.stamp), 1)
    ready, splines = [], []
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
    turn_clear = []
    node.create_subscription(PoseStamped, '/scan/terminal_turn_clear',
                             lambda m: turn_clear.__setitem__(slice(None), [(time.monotonic(), m.header)]), 1)
    latest = []
    def collect_trajectory(message):
        latest[:] = [message]
        if message.pos_pts and any(math.dist((point.x, point.y, point.z),
                                             (message.pos_pts[0].x, message.pos_pts[0].y, message.pos_pts[0].z)) > 1e-8
                                   for point in message.pos_pts):
            splines.append(message)

    node.create_subscription(Bspline, '/d1_trial/bspline', collect_trajectory, 10)
    def submit_route(stop_at=math.inf):
        until = min(stop_at, time.monotonic() + 12)
        while time.monotonic() < until:
            check_planner()
            if recovering and (not clearance.ready() or not ready or not ready[-1]):
                raise RuntimeError('恢复期间失去定位或实时障碍观测')
            if armed:
                output.publish(Twist())
                if losses or not ready[-1]:
                    raise RuntimeError("重规划期间失去定位或运动授权")
            rclpy.spin_once(node, timeout_sec=.15)
            if not clearance.ready() or not ready or not ready[-1] or not publisher.get_subscription_count() or not goal_pub.get_subscription_count():
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
            if time.monotonic() >= stop_at:
                raise TemporarilyBlocked('等待可执行路线超时')
            raise RuntimeError('等待有效定位、障碍点云或规划跟踪节点超时')
        if not recovering:
            print(f"点云、局部地图和位姿就绪：启动后 {time.monotonic()-started:.2f} 秒", flush=True)
        start = anchored.transform.translation
        heading = anchored.transform.rotation
        direction = math.atan2(2 * (heading.w * heading.z + heading.x * heading.y),
                               1 - 2 * (heading.y**2 + heading.z**2))
        translation, rotation = map_to_odom.transform.translation, map_to_odom.transform.rotation
        convert = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
        live_map_points = convert.inv().apply(clearance.observed_obstacles() -
                                              [translation.x, translation.y, translation.z])
        route = path_from_map(np.array([start.x, start.y]), direction, options.goal, live_map_points[:, :2])
        positions = convert.apply(np.column_stack((route, np.full(len(route), start.z))))
        positions += [translation.x, translation.y, translation.z]
        positions[:, 2] = odom_base.transform.translation.z
        message = RosPath()
        message.header.frame_id = 'odom'
        message.header.stamp = node.get_clock().now().to_msg()
        for x, y, z in positions:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
            pose.pose.orientation.w = 1.
            message.poses.append(pose)
        route_length = float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())
        print(json.dumps({'start': route[0].tolist(), 'goal': route[-1].tolist(), 'map_route': route.tolist(),
                          'length_m': route_length}), flush=True)
        endpoint = message.poses[-1]
        yaw = options.yaw + convert.as_euler('xyz')[2]
        endpoint.pose.orientation.w = math.cos(yaw/2)
        endpoint.pose.orientation.z = math.sin(yaw/2)
        commands.clear()
        latest.clear()
        splines.clear()
        goal_pub.publish(endpoint)
        publisher.publish(message)
        print(f"参考路线已下发：启动后 {time.monotonic()-started:.2f} 秒", flush=True)
        until = min(stop_at, time.monotonic() + 12)
        def matching_trajectory():
            return next((spline for spline in splines if spline.goal_stamp == message.header.stamp), None)

        while time.monotonic() < until and matching_trajectory() is None and ready[-1]:
            check_planner()
            if recovering and (not clearance.ready() or not ready or not ready[-1]):
                raise RuntimeError('恢复期间失去定位或实时障碍观测')
            if armed:
                output.publish(Twist())
                if losses or not ready[-1]:
                    raise RuntimeError("重规划期间失去定位或运动授权")
            rclpy.spin_once(node, timeout_sec=.05)
            if latest and latest[0].goal_stamp == message.header.stamp and not any(m is latest[0] for m in splines):
                break
        if latest and latest[0].goal_stamp == message.header.stamp and not any(m is latest[0] for m in splines):
            raise TemporarilyBlocked("规划器已撤回轨迹")
        trajectory = matching_trajectory()
        if ready[-1] and trajectory is None and clearance.ready():
            raise TemporarilyBlocked("正在等待可执行轨迹")
        if trajectory is None or not ready[-1]:
            raise RuntimeError('SCAN did not publish a moving trajectory for this route; inspect scan-preview.log')
        print(f'SCAN moving B-spline {trajectory.traj_id}, control points {len(trajectory.pos_pts)}', flush=True)
        commands.clear()
        return route, route_length, direction, positions, message, endpoint, yaw

    recovering = False
    blocked_since = None
    deadline = math.inf

    def recover_route(reason):
        nonlocal recovering, blocked_since
        recovering = True
        if blocked_since is None:
            blocked_since = time.monotonic()
        print(f'WAITING_FOR_CLEARANCE {reason}', flush=True)
        try:
            stop_at = min(deadline, blocked_since + 30.)
            while time.monotonic() < stop_at:
                output.publish(Twist())
                commands.clear()
                retry_at = time.monotonic() + .25
                while time.monotonic() < retry_at:
                    output.publish(Twist())
                    rclpy.spin_once(node, timeout_sec=.025)
                    check_planner()
                    if losses or not ready[-1] or not clearance.ready():
                        raise RuntimeError('停车等待期间失去授权、定位或障碍观测')
                try:
                    return submit_route(stop_at)
                except TemporarilyBlocked:
                    continue
            raise RuntimeError('等待可执行路线超时，已停车')
        finally:
            recovering = False

    try:
        try:
            result = submit_route()
        except TemporarilyBlocked as exc:
            result = recover_route(exc)
        route, route_length, direction, positions, message, endpoint, yaw = result
        options.goal = route[-1].tolist()
        skip_first_after_arm = False
        waiting_since = None
        shadow_times = []
        last_command = time.monotonic()
        first_velocity_deadline = time.monotonic()+5.
        # The D1 must also turn into the detour and align with the final heading.
        segments = np.diff(route, axis=0)
        segments = segments[np.linalg.norm(segments, axis=1) > 1.e-6]
        headings = np.r_[direction, np.arctan2(segments[:, 1], segments[:, 0]), options.yaw]
        turn_distance = float(np.abs(np.diff(np.unwrap(headings))).sum())
        deadline = time.monotonic() + max(80., 30. + route_length / .06 + turn_distance / .15)
        lateral = 0.
        checked_spline = None
        first_motion_logged = False
        last_check = 0.
        moving_since = None
        pending_recovery = None
        while time.monotonic()<deadline:
            if pending_recovery is not None:
                route, _, _, positions, message, endpoint, yaw = recover_route(pending_recovery)
                pending_recovery = checked_spline = waiting_since = moving_since = None
                lateral = 0.
                skip_first_after_arm = True
                last_command = time.monotonic()
                first_velocity_deadline = last_command + 5.
                continue
            check_planner()
            rclpy.spin_once(node,timeout_sec=.02)
            if not armed and time.monotonic() > first_velocity_deadline:
                raise RuntimeError("已有局部路径，跟踪器 5 秒内未给出可执行速度")
            if losses: raise RuntimeError("运动授权已解除")
            if not ready[-1]:
                raise RuntimeError('定位已丢失')
            if message.header.stamp in acknowledgements:
                print('ARRIVED',flush=True)
                return
            spline = next((v for v in reversed(splines) if v.goal_stamp==message.header.stamp),None)
            terminal = bool(turn_clear and time.monotonic()-turn_clear[0][0] < .15
                            and turn_clear[0][1] == endpoint.header)
            if latest and latest[0].goal_stamp == message.header.stamp and latest[0] is not spline:
                output.publish(Twist())
                pending_recovery = '当前轨迹已撤回'
                continue
            age = (node.get_clock().now().nanoseconds-Time.from_msg(spline.start_time).nanoseconds)*1e-9
            duration = spline.knots[-1-spline.order]-spline.knots[spline.order]
            if not terminal and not -.15 <= age <= duration+.05:
                if waiting_since is None:
                    waiting_since = time.monotonic()
                    print(f'轨迹 {spline.traj_id} 超出有效时间，停车等待本任务的新轨迹',flush=True)
                if time.monotonic()-waiting_since > 3.:
                    pending_recovery = '未收到有效的替代轨迹'
                commands.clear()
                if armed:
                    output.publish(Twist())
                lateral = 0.
                continue
            if waiting_since is not None:
                print(f'轨迹 {spline.traj_id} 已生效，等待新的跟踪指令',flush=True)
                waiting_since = None
                commands.clear()
                skip_first_after_arm = True
                last_command = time.monotonic()
                continue
            if not commands or time.monotonic()-commands[0][0]>.15:
                if armed and time.monotonic()-last_command>.15: raise RuntimeError(f'跟踪速度已过期，间隔 {time.monotonic()-last_command:.3f} 秒')
                continue
            received,cmd = commands.pop()
            if terminal and not -.15 <= age <= duration+.05:
                rotation = Twist()
                rotation.angular.z = cmd.angular.z if cmd.linear.x == cmd.linear.y == 0. else 0.
                cmd = rotation
            if skip_first_after_arm:
                skip_first_after_arm=False
                continue
            last_command = received
            if checked_spline is not spline or time.monotonic() - last_check >= .10:
                gap = clearance.path_clearance(spline, age)
                if gap is None:
                    raise RuntimeError('局部地图或点云未就绪，已停车')
                if gap <= .48:
                    output.publish(Twist())
                    pending_recovery = f'近期轨迹净空 {gap:.3f} m'
                    continue
                checked_spline = spline
                last_check = time.monotonic()
            # At the end of the final spline the tangent can swing by pi
            # across successive replans. Keep tracking translation on the
            # checked spline, but use the user's fixed heading in this zone.
            _, position, rotation = clearance.pose
            goal_xy = positions[-1, :2]
            tail = spline.pos_pts[-1]
            final_leg = math.hypot(tail.x-goal_xy[0], tail.y-goal_xy[1]) < .10
            if final_leg and np.linalg.norm(position[:2]-goal_xy) < .45:
                body_yaw = rotation.as_euler('xyz')[2]
                error = math.atan2(math.sin(yaw-body_yaw), math.cos(yaw-body_yaw))
                translating = bool(cmd.linear.x or cmd.linear.y)
                if translating or (terminal and cmd.angular.z):
                    cmd.angular.z = float(np.clip(error, -.25, .25)) if abs(error) > math.radians(3) else 0.
            cmd.linear.y = sdk_lateral_speed(cmd.linear.y, lateral)
            try:
                clearance.check(cmd)
            except BodyBlocked as exc:
                output.publish(Twist())
                pending_recovery = str(exc)
                continue
            if not armed:
                if max(abs(cmd.linear.x),abs(cmd.linear.y),abs(cmd.angular.z))<.001: continue
                if not options.shadow:
                    future=arm.call_async(SetBool.Request(data=True))
                    rclpy.spin_until_future_complete(node,future,timeout_sec=.5)
                    if not future.done() or not future.result().success:
                        raise RuntimeError(future.result().message if future.done() else '授权超时')
                elif options.shadow_arm_delay:
                    time.sleep(options.shadow_arm_delay)
                skip_first_after_arm=True
                last_command=time.monotonic()
                commands.clear()
                armed=True
                print('TRACKING',flush=True)
                continue
            lateral = cmd.linear.y
            if not first_motion_logged and any((cmd.linear.x, cmd.linear.y, cmd.angular.z)):
                print(f"首条非零指令，任务启动后 {time.monotonic()-started:.2f} 秒", flush=True)
                first_motion_logged = True
            if any((cmd.linear.x, cmd.linear.y, cmd.angular.z)):
                if moving_since is None:
                    moving_since = time.monotonic()
                elif time.monotonic() - moving_since >= 1.:
                    blocked_since = None
            else:
                moving_since = None
            output.publish(cmd)
            if options.shadow and max(abs(cmd.linear.x),abs(cmd.linear.y),abs(cmd.angular.z))>=.05:
                shadow_times.append(time.monotonic())
                if len(shadow_times)>1 and shadow_times[-1]-shadow_times[0]>=1.5:
                    print('SHADOW',len(shadow_times),'samples; max interval', max(np.diff(shadow_times)),flush=True)
                    return
        raise RuntimeError('到点超时')
    finally:
        if armed and not options.shadow:
            output.publish(Twist())
            future=arm.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(node,future,timeout_sec=.4)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
