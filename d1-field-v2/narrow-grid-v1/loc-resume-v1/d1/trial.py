from clearance import BodyClearance, BodyBlocked, StaticBoundary
from map_route import load_surface, path_from_map
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
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


def sdk_lateral_speed(requested, previous, limit=.12):
    # Match the D1 SDK's nonzero side-step range before the bridge clamps it.
    threshold = .015 if previous * requested > 0. else .030
    if abs(requested) < threshold:
        return 0.
    return math.copysign(min(limit, max(.10, abs(requested))), requested)


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
    forward_max = float(os.environ.get("D1_FORWARD_MAX", ".12"))
    yaw_max = float(os.environ.get("D1_YAW_MAX", ".25"))
    lateral_max = float(os.environ.get("D1_LATERAL_MAX", ".12"))
    clearance = BodyClearance(node, forward_max, yaw_max, lateral_max)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    _, _, surface, center, resolution, offset = load_surface()
    def on_static_surface(points):
        try:
            tf = buffer.lookup_transform('map', 'odom', Time())
        except TransformException as exc:
            raise RuntimeError('静态地图坐标变换不可用') from exc
        t, q = tf.transform.translation, tf.transform.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        xyz = np.column_stack((points, np.full(len(points), clearance.pose[1][2])))
        xy = rotation.apply(xyz)[:, :2] + [t.x, t.y]
        indices = np.rint((xy-center)/resolution).astype(int)+offset
        return (np.all(indices >= 0) and np.all(indices < surface.shape)
                and surface[tuple(indices.T)].all())
    clearance.on_static_surface = on_static_surface
    publisher = node.create_publisher(RosPath, '/d1_community/initial_path',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    output = node.create_publisher(Twist, '/navigation/approved_velocity' if not options.shadow else '/d1_community/shadow_velocity', 1)
    arm = node.create_client(SetBool, '/d1/arm_motion')
    resume = node.create_client(SetBool, '/d1/resume_motion')
    latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    recovery_allowed = [False]
    node.create_subscription(Bool, '/d1/recovery_allowed',
                             lambda m: recovery_allowed.__setitem__(0, m.data), latched)
    fix = {}
    mismatch = [None]
    def on_status(msg):
        data = json.loads(msg.data)
        fix.clear()
        fix.update(data)
        if data.get('state') == 'COLLECTING' and data.get('reason') != 'MAP_TRACKING_MISMATCH':
            mismatch[0] = None
        if data.get('automatic') and data.get('reason') == 'MAP_TRACKING_MISMATCH':
            mismatch[0] = (data.get('request_id'), time.monotonic())
    node.create_subscription(String, '/localization/status', on_status, latched)
    commands = []
    losses = []
    node.create_subscription(Bool, '/d1/motion_armed', lambda m: losses.append(True) if armed and not options.shadow and not m.data else None, 1)
    last_candidate = [0.]
    raw_sample, applied_sample = [], []
    def remember(sample, msg):
        sample[:] = [time.monotonic(), msg.linear.x, msg.linear.y, msg.angular.z]
    applied_motion_logged = applied_translation_logged = False
    def on_applied(msg):
        nonlocal applied_motion_logged, applied_translation_logged
        remember(applied_sample, msg)
        if not armed:
            return
        stamp, vx, vy, yaw_rate = applied_sample
        if not applied_motion_logged and any((vx, vy, yaw_rate)):
            applied_motion_logged = True
            print(f"控制桥首次执行非零指令 {stamp-started:.3f}s", flush=True)
        if not applied_translation_logged and (vx or vy):
            applied_translation_logged = True
            print(f"控制桥首次执行平移指令 {stamp-started:.3f}s", flush=True)
    node.create_subscription(Twist, '/d1/velocity_applied', on_applied, 1)
    def candidate(msg):
        last_candidate[0] = time.monotonic()
        remember(raw_sample, msg)
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
    route_reference = None
    route_request = None
    def submit_route(validate=None):
        nonlocal route_reference, route_request
        until = time.monotonic() + 12
        while time.monotonic() < until:
            check_planner()
            rclpy.spin_once(node, timeout_sec=.15)
            if armed and (losses or (ready and not ready[-1])):
                return None  # Let the task loop handle loss before changing its reference pose.
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
        if validate:
            validate(map_to_odom, odom_base)
        route_reference = map_to_odom
        route_request = fix.get('request_id')
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
        print(f"路线发布 {time.monotonic()-started:.3f}s", flush=True)
        publisher.publish(message)
        return positions[-1,:2], yaw, message.header.stamp

    try:
        goal_xy, yaw, stamp = submit_route()
        origin_xy = clearance.pose[1][:2].copy()
        last_odom_stamp = clearance.pose[0]
        displacement_frames = 0
        motion_observed = False
        first_stamp = Time.from_msg(stamp).nanoseconds
        deadline = time.monotonic() + 180.
        grid_wait_since = None
        last_reason = None
        logged_at = 0.
        lateral = forward = 0.
        settled = None
        first_velocity = False
        last_route_request = time.monotonic()
        checked = None
        boundary_since = None
        boundary_replanned = False
        checked_at = 0.
        gap = 0.
        resuming = False
        def validate_fix(transform, odom_base):
            def as_pose(tf):
                v, q = tf.transform.translation, tf.transform.rotation
                return np.array([v.x,v.y,v.z]), Rotation.from_quat([q.x,q.y,q.z,q.w])
            pt, _ = as_pose(odom_base)
            old, rot0 = as_pose(route_reference)
            new, rot1 = as_pose(transform)
            delta = rot1.inv().apply(pt-new) - rot0.inv().apply(pt-old)
            angle = (rot1.inv()*rot0).magnitude()
            if not np.isfinite(delta).all() or np.linalg.norm(delta) > .30 or angle > math.radians(15):
                raise RuntimeError('重新定位修正过大，请确认地图位姿后重新发起任务')
            if (not recovery_allowed[0] or fix.get('request_id') != route_request or
                    fix.get('state') != 'LOCALIZED' or not fix.get('automatic') or
                    Time.from_msg(transform.header.stamp).nanoseconds < fix.get('fix_stamp_ns', 0)):
                raise RuntimeError('恢复授权或定位结果发生变化')

        while time.monotonic() < deadline:
            check_planner()
            rclpy.spin_once(node, timeout_sec=.015)
            if losses or not ready[-1]:
                output.publish(Twist())
                if not armed or options.shadow:
                    raise RuntimeError('定位或运动授权失效')
                armed = False
                print('WAIT_LOCALIZATION', flush=True)
                stopped = time.monotonic()
                loss_stamp = node.get_clock().now().nanoseconds
                stable_since = None
                last_stamp = 0
                _, stopped_position, stopped_rotation = clearance.pose
                permit_seen = False
                while time.monotonic() < min(deadline, stopped+15.):
                    check_planner()
                    rclpy.spin_once(node, timeout_sec=.05)
                    elapsed = time.monotonic() - stopped
                    if not clearance.ready():
                        raise RuntimeError('等待定位期间障碍观测或里程计失效')
                    pose_time, pos, rot = clearance.pose
                    pose_age = node.get_clock().now().nanoseconds - pose_time
                    if not 0 <= pose_age < 350_000_000:
                        raise RuntimeError('等待定位期间机身里程计过期')
                    if (np.linalg.norm(pos-stopped_position) > .30 or
                            (rot.inv()*stopped_rotation).magnitude() > math.radians(15)):
                        raise RuntimeError('等待定位期间机身位姿变化过大')
                    permit_seen |= recovery_allowed[0]
                    if not recovery_allowed[0] and (permit_seen or elapsed > .7):
                        raise RuntimeError('控制桥未允许自动恢复，任务已停止')
                    if fix.get('state') in ('FAILED','LOST') or fix.get('request_id') != route_request:
                        raise RuntimeError('定位失败或已手动重设初始位姿')
                    if not (mismatch[0] and mismatch[0][0] == route_request and
                            mismatch[0][1] >= stopped-.7 and fix.get('automatic') and
                            fix.get('state') == 'LOCALIZED' and ready[-1] and
                            fix.get('fix_stamp_ns', 0) >= loss_stamp and recovery_allowed[0]):
                        stable_since = None
                        continue
                    try:
                        tf = buffer.lookup_transform('odom','map',Time())
                        base = buffer.lookup_transform('odom','base_link',Time())
                    except TransformException:
                        stable_since = None
                        continue
                    stamp_ns = Time.from_msg(tf.header.stamp).nanoseconds
                    age_ns = node.get_clock().now().nanoseconds-stamp_ns
                    if stamp_ns < fix['fix_stamp_ns'] or not 0 <= age_ns < 350_000_000:
                        stable_since = None
                        continue
                    validate_fix(tf, base)
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif stamp_ns > last_stamp and time.monotonic()-stable_since >= 1.:
                        break
                    last_stamp = stamp_ns
                else:
                    raise RuntimeError('等待稳定定位超过时限，任务已停止')
                losses.clear()
                latest.clear()
                commands.clear()
                lateral = forward = 0.
                moving[0] = False
                checked = settled = None
                print('REPLAN_LOCALIZED', flush=True)
                goal_xy, yaw, stamp = submit_route(validate_fix)
                first_stamp = Time.from_msg(stamp).nanoseconds
                mismatch[0] = None
                last_route_request = time.monotonic()
                resuming = True
                continue
            now = time.monotonic()
            pose_stamp, position, rotation = clearance.pose
            ros_now = node.get_clock().now().nanoseconds
            if not 0 <= ros_now - pose_stamp < 350_000_000:
                raise RuntimeError('机身里程计过期')
            if not clearance.sensors_ready():
                raise RuntimeError('实时障碍点云或雷达里程计失效')
            if not clearance.map_ready():
                output.publish(Twist())
                if grid_wait_since is None:
                    grid_wait_since = now
                    print('局部栅格断更，停车等待更新', flush=True)
                if now-grid_wait_since >= 5.:
                    raise RuntimeError('局部栅格持续断更，已停车')
                latest.clear()
                commands.clear()
                moving[0] = False
                checked = settled = None
                lateral = forward = 0.
                continue
            if grid_wait_since is not None:
                output.publish(Twist())
                latest.clear()
                commands.clear()
                moving[0] = False
                grid_wait_since = None
                new_route = submit_route()
                if new_route is None:
                    continue
                goal_xy, yaw, stamp = new_route
                first_stamp = Time.from_msg(stamp).nanoseconds
                last_route_request = time.monotonic()
                print('局部栅格恢复，已从当前位姿重新请求路线', flush=True)
                continue
            if armed and not motion_observed and pose_stamp > last_odom_stamp:
                last_odom_stamp = pose_stamp
                displacement = np.linalg.norm(position[:2] - origin_xy)
                displacement_frames = displacement_frames + 1 if displacement >= .03 else 0
                if displacement_frames >= 2:
                    motion_observed = True
                    print(f"里程计连续两帧平移超过 3 cm {now-started:.3f}s", flush=True)
            if boundary_since is not None:
                if now-boundary_since >= 6.:
                    raise RuntimeError(str(StaticBoundary()))
                if now-boundary_since >= 1. and not boundary_replanned:
                    output.publish(Twist())
                    latest.clear()
                    commands.clear()
                    moving[0] = False
                    checked = None
                    lateral = forward = 0.
                    boundary_replanned = True
                    new_route = submit_route()
                    if new_route is not None:
                        goal_xy, yaw, stamp = new_route
                        first_stamp = Time.from_msg(stamp).nanoseconds
                        last_route_request = time.monotonic()
                    continue
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
                    cmd.angular.z = float(np.clip(error,-yaw_max,yaw_max))
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
                    if age > duration + .5 and now - last_route_request > 2.:
                        # The tracker can freeze spline time for heading adjustment, while
                        # the guard expires that spline in wall time. Replace the route
                        # from fresh odometry; never forward a command from the old spline.
                        output.publish(Twist())
                        latest.clear()
                        commands.clear()
                        checked = None
                        lateral = forward = 0.
                        moving[0] = False
                        print("轨迹已到期，按实测位姿重新规划", flush=True)
                        new_route = submit_route()
                        if new_route is None:
                            continue
                        goal_xy, yaw, stamp = new_route
                        first_stamp = Time.from_msg(stamp).nanoseconds
                        last_route_request = time.monotonic()
                        continue
                    if -.05 <= age < duration:
                        if checked is not spline or now-checked_at >= .10:
                            try:
                                gap = clearance.path_clearance(spline,age)
                            except StaticBoundary:
                                gap = None
                                if boundary_since is None:
                                    boundary_since = now
                                    print("局部绕行将越出静态可通行区，停车等待新轨迹", flush=True)
                            checked, checked_at = spline, now
                        if gap is not None and gap > 0.:
                            cmd = candidate_cmd
                            reason = '分支跟踪器输出零速度'
                        else:
                            reason = f'D1 近期轨迹净空 {gap} m'
            elif not commands and now-last_candidate[0] < .12 and latest and moving[0]:
                # No command callback in this spin. The bridge keeps its own short watchdog.
                continue
            if not np.isfinite([cmd.linear.x,cmd.linear.y,cmd.angular.z]).all():
                raise RuntimeError('控制器输出非有限速度')
            raw = (cmd.linear.x, cmd.linear.y, cmd.angular.z)
            threshold = .006 if forward*cmd.linear.x > 0. else min(.055, forward_max)
            cmd.linear.x = (math.copysign(np.clip(abs(cmd.linear.x), .05, forward_max), cmd.linear.x)
                            if abs(cmd.linear.x) >= threshold else 0.)
            cmd.linear.y = sdk_lateral_speed(cmd.linear.y,lateral,lateral_max)
            cmd.angular.z = float(np.clip(cmd.angular.z,-yaw_max,yaw_max))
            # A tiny heading trim during a translational ramp otherwise initiates
            # a spin/stop before the D1 can sustain its first forward step.
            waiting_for_step = (not first_velocity and not at_goal and not forward and not lateral
                                and (raw[0] or raw[1]) and not cmd.linear.x and not cmd.linear.y)
            if abs(cmd.angular.z) < (.06 if waiting_for_step else .02):
                cmd.angular.z = 0.
            try:
                clearance.check(cmd)
            except StaticBoundary:
                if boundary_since is None:
                    boundary_since = now
                    print("底盘运动将越出静态可通行区，停车等待新轨迹", flush=True)
                cmd = Twist()
                reason = '静态地图边界保护'
            except BodyBlocked:
                cmd = Twist()
                reason = 'D1 机身刹停保护'
            if boundary_since is not None and not (cmd.linear.x or cmd.linear.y):
                cmd = Twist()
                reason = '等待静态可通行区内的新轨迹'
            active = any((cmd.linear.x,cmd.linear.y,cmd.angular.z))
            if (cmd.linear.x or cmd.linear.y) and gap is not None and gap > 0.:
                boundary_since = None
                boundary_replanned = False
            reason = '运动' if active else reason
            if reason.startswith('D1 近期'):
                reason = f'D1 近期轨迹净空不足（{gap:.2f} m）' if gap is not None else '局部地图未就绪'
            if not active and reason == '分支跟踪器输出零速度' and any(raw):
                reason = '候选速度低于底盘可执行门槛'
            if reason != last_reason and now-logged_at > .25:
                print(json.dumps(dict(t=now, candidate=raw_sample, approved=[cmd.linear.x, cmd.linear.y, cmd.angular.z],
                                      last_bridge_send=applied_sample, reason=reason), ensure_ascii=False),flush=True)
                print(f'试验 {now-started:.2f}s {reason}',flush=True)
                last_reason, logged_at = reason, now
            if not armed and active:
                if not options.shadow:
                    client = resume if resuming else arm
                    fut=client.call_async(SetBool.Request(data=True))
                    rclpy.spin_until_future_complete(node,fut,timeout_sec=.5)
                    if not fut.done() or not fut.result().success:
                        raise RuntimeError('D1 控制桥未授权')
                armed=True
                resuming=False
                commands.clear()
                print('TRACKING',flush=True)
                continue
            if armed:
                output.publish(cmd)
            if os.environ.get('D1_VELOCITY_TRACE') == '1':
                print("PAIR",now,*raw,"OUT",cmd.linear.x,cmd.linear.y,cmd.angular.z,reason,flush=True)
            lateral, forward = cmd.linear.y, cmd.linear.x
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
