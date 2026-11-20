"""Exercise the GO2 heading reference across replans, clearance changes and corners."""
import math
import os
import subprocess
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point, Vector3Stamped
from nav_msgs.msg import Odometry
from scan_planner.msg import Bspline
from std_msgs.msg import Int64


def run():
    rclpy.init()
    node = rclpy.create_node('heading_regression')
    prefix = f'/heading_test_{os.getpid()}'
    odometry = node.create_publisher(Odometry, prefix + '/odom', 10)
    trajectory = node.create_publisher(Bspline, prefix + '/spline', 10)
    clearance = node.create_publisher(Int64, prefix + '/clear', 1)
    readings = []
    node.create_subscription(Vector3Stamped, prefix + '/ref', readings.append, 10)
    executable = Path(get_package_prefix('scan_planner')) / 'lib/scan_planner/scan_go2_cmd_adapter'
    process = subprocess.Popen([str(executable), '--ros-args', '-p', 'body_pose_topic:=' + prefix + '/odom',
                                '-r', '/scan/planning/bspline:=' + prefix + '/spline',
                                '-r', '/scan/heading_clear_trajectory:=' + prefix + '/clear',
                                '-r', '/scan/tracking_heading:=' + prefix + '/ref'])

    def sample(index, tangent, permit, body=0.0):
        message = Bspline(order=3, traj_id=index, start_time=node.get_clock().now().to_msg(),
                          knots=[0.5 * knot for knot in range(-3, 11)])
        message.pos_pts = [Point(x=0.2 * step * math.cos(tangent), y=0.2 * step * math.sin(tangent))
                           for step in range(-1, 9)]
        trajectory.publish(message)
        pose = Odometry()
        pose.header.frame_id = 'odom'
        pose.pose.pose.orientation.z = math.sin(body / 2)
        pose.pose.pose.orientation.w = math.cos(body / 2)
        readings.clear()
        until = time.monotonic() + 0.35
        while time.monotonic() < until:
            pose.header.stamp = node.get_clock().now().to_msg()
            odometry.publish(pose)
            if permit is not None:
                clearance.publish(Int64(data=permit))
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.002)
        assert readings and readings[-1].header.frame_id == 'odom'
        return readings[-1].vector

    try:
        end = time.monotonic() + 10
        while not all(pub.get_subscription_count() for pub in (odometry, trajectory, clearance)) or not node.count_publishers(prefix + '/ref'):
            assert process.poll() is None and time.monotonic() < end, 'discovery timed out'
            rclpy.spin_once(node, timeout_sec=0.05)
        for index in range(1, 7):
            angle = (-1)**index * 0.14
            result = sample(index, angle, index)
            assert abs(result.x - angle) < 0.001 and abs(result.y) < 0.075, result
        for index, angle, permit in [(7, 0.14, 6), (8, -0.14, -1), (9, 0.95, 9)]:
            result = sample(index, angle, permit)
            assert abs(result.y - angle) < 0.001, result
        sample(10, 0.14, 10)
        result = sample(10, 0.14, None)
        assert abs(result.y - result.x) < 0.001, 'expired clearance'
        result = sample(11, -math.pi + 0.03, 11, math.pi - 0.02)
        assert abs(math.remainder(result.y - math.pi, 2 * math.pi)) < 0.08, result
        print('PASS alternating splines, stale/wrong/revoked clearance, corner and angle wrap')
    finally:
        process.terminate()
        try:
            process.wait(timeout=6)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run()
