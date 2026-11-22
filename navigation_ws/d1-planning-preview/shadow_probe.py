from preview import path_from_map
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


def main():
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
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(RosPath, '/d1_trial/initial_path',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    goal_pub = node.create_publisher(PoseStamped, '/scan/goal', 10)
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
    node.create_subscription(Bool, '/localization/ready', lambda msg: ready.append(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    latest = []
    def collect_trajectory(message):
        latest[:] = [message]
        if message.pos_pts and any(math.dist((point.x, point.y, point.z),
                                             (message.pos_pts[0].x, message.pos_pts[0].y, message.pos_pts[0].z)) > .05
                                   for point in message.pos_pts):
            splines.append(message)

    node.create_subscription(Bspline, '/d1_trial/bspline', collect_trajectory, 10)
    try:
        until = time.monotonic() + 12
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.15)
            if not ready or not ready[-1] or publisher.get_subscription_count() < 1:
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
            raise RuntimeError('Waiting for a fresh D1 map pose and SCAN subscription')
        start = anchored.transform.translation
        heading = anchored.transform.rotation
        direction = math.atan2(2 * (heading.w * heading.z + heading.x * heading.y),
                               1 - 2 * (heading.y**2 + heading.z**2))
        route = path_from_map(np.array([start.x, start.y]), direction, options.goal)
        translation, rotation = map_to_odom.transform.translation, map_to_odom.transform.rotation
        convert = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
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
        print(json.dumps({'start': route[0].tolist(), 'goal': route[-1].tolist(), 'map_route': route.tolist(),
                          'length_m': float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())}), flush=True)
        if np.linalg.norm(np.diff(route,axis=0),axis=1).sum() > 2.5:
            raise RuntimeError('首次真机试验路线限长 2.5 m')
        endpoint = message.poses[-1]
        yaw = options.yaw + convert.as_euler('xyz')[2]
        endpoint.pose.orientation.w = math.cos(yaw/2)
        endpoint.pose.orientation.z = math.sin(yaw/2)
        goal_pub.publish(endpoint)
        publisher.publish(message)
        until = time.monotonic() + 12
        def matching_trajectory():
            return next((spline for spline in splines if spline.goal_stamp == message.header.stamp), None)

        while time.monotonic() < until and matching_trajectory() is None and ready[-1]:
            rclpy.spin_once(node, timeout_sec=.15)
        trajectory = matching_trajectory()
        if trajectory is None or not ready[-1]:
            raise RuntimeError('SCAN did not publish a moving trajectory for this route; inspect scan-preview.log')
        print(f'SCAN moving B-spline {trajectory.traj_id}, control points {len(trajectory.pos_pts)}', flush=True)
        skip_first_after_arm = False
        shadow_times = []
        last_command = time.monotonic()
        deadline = time.monotonic()+80
        parent_pid = int(os.getenv('D1_TASK_PARENT','0'))
        while time.monotonic()<deadline:
            if parent_pid and not os.path.exists(f'/proc/{parent_pid}'):
                raise RuntimeError('网页控制服务已退出')
            rclpy.spin_once(node,timeout_sec=.02)
            if losses: raise RuntimeError("运动授权已解除")
            if not ready[-1]:
                raise RuntimeError('定位已丢失')
            if message.header.stamp in acknowledgements:
                print('ARRIVED',flush=True)
                return
            spline = next((v for v in reversed(splines) if v.goal_stamp==message.header.stamp),None)
            if latest and latest[0] is not spline:
                raise RuntimeError('局部规划器已撤销当前运动轨迹')
            age = (node.get_clock().now().nanoseconds-Time.from_msg(spline.start_time).nanoseconds)*1e-9
            duration = spline.knots[-1-spline.order]-spline.knots[spline.order]
            if not -.15 <= age <= duration+.05:
                raise RuntimeError('局部轨迹已过期')
            if not commands or time.monotonic()-commands[0][0]>.15:
                if armed and time.monotonic()-last_command>.15: raise RuntimeError(f'跟踪速度已过期，间隔 {time.monotonic()-last_command:.3f} 秒')
                continue
            received,cmd = commands.pop()
            if skip_first_after_arm:
                skip_first_after_arm=False
                continue
            last_command = received
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
