"""Exercise the SCAN adapter's terminal hold with timestamped ROS 2 odometry."""

import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from scan_planner.msg import Bspline
from std_msgs.msg import Bool


def run():
    rclpy.init()
    node = rclpy.create_node('test_goal_settling')
    prefix = '/test_goal_settling_' + str(os.getpid())
    clock_pub = node.create_publisher(Clock, prefix + '/clock', 10)
    odom_pub = node.create_publisher(Odometry, prefix + '/odom', 10)
    goal_pub = node.create_publisher(PoseStamped, prefix + '/goal', 10)
    spline_pub = node.create_publisher(Bspline, prefix + '/spline', 10)
    acknowledgments = []
    commands = []
    frozen = []
    node.create_subscription(PoseStamped, prefix + '/ack', acknowledgments.append, 10)
    node.create_subscription(Twist, prefix + '/cmd', commands.append, 10)
    node.create_subscription(Bool, prefix + '/frozen', frozen.append, 10)
    executable = Path(get_package_prefix('scan_planner')) / 'lib/scan_planner/scan_go2_cmd_adapter'
    adapter = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'use_sim_time:=true', '-p', 'goal_settle_time:=0.6',
        '-p', 'require_route_stamp:=true',
        '-p', 'finish_dist:=0.07', '-p', 'min_walk_speed:=0.12', '-p', 'goal_min_walk_speed:=0.20',
        '-p', 'use_goal_z:=true', '-p', 'finish_dist_z:=0.8',
        '-p', 'body_pose_topic:=' + prefix + '/odom', '-r', '/clock:=' + prefix + '/clock',
        '-r', '/scan/goal:=' + prefix + '/goal', '-r', '/scan/goal_stop_ack:=' + prefix + '/ack',
        '-r', '/scan/planning/bspline:=' + prefix + '/spline', '-r', '/cmd_vel:=' + prefix + '/cmd',
        '-r', '/scan/planning/execution_frozen:=' + prefix + '/frozen'])
    stamp_ns = 10_000_000_000

    def spin_for(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)

    def sample(x, height=1., stamp=None):
        nonlocal stamp_ns
        stamp_ns += 100_000_000
        current = Time(nanoseconds=stamp_ns).to_msg()
        clock_pub.publish(Clock(clock=current))
        message = Odometry()
        message.header.frame_id = 'odom'
        message.header.stamp = current if stamp is None else stamp
        message.pose.pose.orientation.w = 1.
        message.pose.pose.position.x = x
        message.pose.pose.position.z = height
        odom_pub.publish(message)
        spin_for(0.05)
        return current

    try:
        deadline = time.monotonic() + 10.
        while not (clock_pub.get_subscription_count() and odom_pub.get_subscription_count()
                   and goal_pub.get_subscription_count() and spline_pub.get_subscription_count()
                   and node.count_publishers(prefix + '/ack') and node.count_publishers(prefix + '/cmd')):
            assert adapter.poll() is None and time.monotonic() < deadline, 'adapter discovery timed out'
            spin_for(0.05)

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = sample(-0.5)
        goal.pose.position.z = 1.
        goal_pub.publish(goal)
        spin_for(0.05)
        spline = Bspline(order=3, knots=[float(index) for index in range(-3, 7)],
                         start_time=goal.header.stamp,
                         pos_pts=[Point(x=value, z=1.) for value in (-0.7, -0.5, -0.3, 0., 0., 0.)])
        spline_pub.publish(spline)
        for _ in range(3):
            sample(-0.5)
        assert commands and not any(command.linear.x for command in commands), 'foreign segment started motion'
        spline.goal_stamp = goal.header.stamp
        spline.start_time = sample(-0.5)
        spline_pub.publish(spline)
        for _ in range(3):
            sample(-0.5)
        assert any(command.linear.x > 0.001 for command in commands), 'moving spline produced no motion'
        for _ in range(8):
            sample(0.068)
        commands.clear()
        for _ in range(4):
            sample(0.073)
            sample(0.067)
        speeds = [command.linear.x for command in commands]
        assert speeds and max(speeds) < -0.06, f'outer-edge noise interrupted approach: {speeds}'
        assert speeds[-1] < -0.19, f'terminal speed did not ramp to the configured minimum: {speeds}'
        assert not acknowledgments and not frozen[-1].data
        sample(0.04)
        sample(0.058)
        sample(0.058)
        assert not acknowledgments and frozen[-1].data and commands[-1] == Twist(), 'drift in the outer band released braking'
        sample(0.073)
        sample(0.073)
        assert not frozen[-1].data and commands[-1].linear.x < 0., 'leaving the outer radius did not resume the spline'

        spline.start_time = Time(nanoseconds=stamp_ns).to_msg()
        spline.pos_pts = [Point(x=-0.5, z=1.) for _ in range(6)]
        spline_pub.publish(spline)
        spin_for(0.05)
        commands.clear()
        for _ in range(3):
            sample(-0.25)
        assert commands and all(command == Twist() for command in commands), 'emergency spline drove back to its anchor'

        first = sample(0.04)
        for _ in range(9):
            sample(0.04, stamp=first)
        assert not acknowledgments, 'repeated odometry started or completed the dwell'

        for _ in range(4):
            sample(0.04)
            sample(-0.04)
        assert not acknowledgments, 'a moving base passed the terminal hold'
        for _ in range(9):
            sample(0.04, height=4.)
        assert not acknowledgments, 'a different floor passed the terminal hold'

        for _ in range(5):
            sample(0.04)
        sample(0.09)
        for _ in range(5):
            sample(0.04)
        assert not acknowledgments, 'leaving the tolerance did not restart the dwell'
        for _ in range(6):
            sample(0.058)
        assert len(acknowledgments) == 1, 'a stable in-tolerance goal must receive one acknowledgment'
        assert acknowledgments[0].header == goal.header and not frozen[-1].data
        assert acknowledgments[0].pose.position.x == 0.058
        print('PASS approach under edge jitter, braking hysteresis, emergency stop, and fresh same-floor stable arrival')
    finally:
        adapter.terminate()
        adapter.wait(timeout=5.)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
