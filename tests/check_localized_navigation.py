"""Check localized PCT start, frame conversion, cancellation and command authorization on ROS 2."""
import json
import math
import runpy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.time import Time
from scan_planner.msg import Bspline
from std_msgs.msg import Bool, Header, Int32, String


scripts = Path(__file__).resolve().parents[1] / 'src/legbot_bringup/scripts'
Mission = runpy.run_path(str(scripts / 'pct_path_mission'))['PctPathMission']
Adapter = runpy.run_path(str(scripts / 'go2_command_adapter'))['CommandAdapter']


def run():
    rclpy.init(args=['--ros-args', '-p', 'use_map_localization:=true', '-p', 'pct_frame_id:=map',
                     '-p', 'require_map_localization:=true', '-p', 'require_navigation_data:=false'])
    mission, adapter = Mission(), Adapter()
    now = [10_000_000_000]
    mission.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=now[0]))
    for name in ('start_pub', 'permit_pub', 'plan_ready_pub', 'path_pub', 'goal_pub', 'complete_pub'):
        setattr(mission, name, Mock())
    adapter.pub = Mock()

    def status(state, fix=None):
        message = String(data=json.dumps(dict(state=state, fix_stamp_ns=fix)))
        mission.on_localization(message)
        adapter.on_fix(message)

    def send_tf(stamp_ns):
        transform = TransformStamped()
        transform.header = Header(frame_id='map', stamp=Time(nanoseconds=stamp_ns).to_msg())
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = 10.
        transform.transform.translation.y = -5.
        transform.transform.translation.z = 1.
        transform.transform.rotation.z = math.sin(math.pi / 4)
        transform.transform.rotation.w = math.cos(math.pi / 4)
        mission.buffer.set_transform(transform, 'test_localizer')

    def check_command(walking):
        command = Twist()
        command.linear.x = 0.3
        adapter.control_mode(Int32(data=3))
        adapter.command(command)
        adapter.publish()
        output = adapter.pub.publish.call_args.args[0]
        assert math.isclose(output.ly, 0.3 if walking else 0.0, abs_tol=1e-6)
        assert output.command == (3 if walking else 0)

    try:
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.header.stamp = Time(nanoseconds=now[0]).to_msg()
        odom.pose.pose.position = Point(x=1., y=2., z=0.5)
        odom.pose.pose.orientation.w = 1.
        mission.on_odom(odom)
        mission.on_ready(Bool(data=True))
        status('COLLECTING')
        assert not mission.start_pub.publish.called
        check_command(False)

        send_tf(now[0] - 1)
        status('LOCALIZED', now[0])
        adapter.on_ready(Bool(data=True))
        assert not mission.start_pub.publish.called
        send_tf(now[0])
        mission.tick_localized()
        start = mission.start_pub.publish.call_args.args[0]
        assert start.header.frame_id == 'map' and start.header.stamp == odom.header.stamp
        assert math.dist((start.pose.position.x, start.pose.position.y, start.pose.position.z), (8., -4., 1.5)) < 1e-9
        check_command(False)

        path = RosPath(header=Header(frame_id='map'))
        goal = PoseStamped()
        goal.pose.position = Point(x=9., y=-4., z=3.5)
        path.poses = [start, goal]
        mission.on_path(path)
        assert not mission.started
        path.header.stamp = start.header.stamp
        now[0] += 100_000_000
        mission.on_path(path)
        assert mission.started
        endpoint = mission.path_pub.publish.call_args.args[0].poses[-1].pose.position
        assert math.dist((endpoint.x, endpoint.y, endpoint.z), (1., 1., 2.5)) < 1e-9
        spline = Bspline(order=1, knots=[0., 0., 1., 1.], start_time=start.header.stamp,
                         pos_pts=[Point(), Point(x=1.)])
        mission.on_plan(spline)
        assert not mission.plan_accepted
        spline.start_time = mission.goal_pub.publish.call_args.args[0].header.stamp
        mission.on_plan(spline)
        assert mission.plan_accepted
        permit = mission.permit_pub.publish.call_args.args[0]
        adapter.on_permit(permit)
        check_command(True)
        adapter.last_permit -= adapter.navigation_timeout + 1.
        check_command(False)

        adapter.on_ready(Bool(data=False))
        status('FAILED')
        assert not mission.started and not mission.path_pub.publish.call_args.args[0].poses
        assert not mission.start_pub.publish.call_args.args[0].header.frame_id
        check_command(False)
        now[0] += 1_000_000_000
        status('LOCALIZED', now[0])
        adapter.on_ready(Bool(data=True))
        adapter.on_permit(permit)
        mission.on_path(path)
        assert not mission.started and mission.request_stamp is None
        check_command(False)
        send_tf(now[0])
        odom.header.stamp = Time(nanoseconds=now[0]).to_msg()
        mission.on_odom(odom)
        assert mission.request_stamp == odom.header.stamp
        mission.on_path(path)
        assert not mission.started
        path.header.stamp = mission.request_stamp
        mission.on_path(path)
        assert mission.started and not mission.plan_accepted
        mission.on_plan(spline)
        assert not mission.plan_accepted
        final_goal = mission.goal_pub.publish.call_args.args[0]
        mission.on_stop_ack(final_goal)
        assert mission.completed and not mission.permit_pub.publish.call_args.args[0].frame_id
        print('PASS corrected map start, stale TF/path/spline rejection, route conversion, stop, replan, command gate')
    finally:
        adapter.destroy_node()
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
