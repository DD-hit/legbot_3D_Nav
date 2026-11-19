"""Exercise initial-pose request lifetimes and stamped TF on ROS 2 without Gazebo."""

import importlib.util
import json
import math
import sys
from concurrent.futures import Future
from importlib.machinery import SourceFileLoader
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.clock import ClockType
from rclpy.time import Time
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


scripts = Path(__file__).resolve().parents[1] / 'src/legbot_bringup/scripts'
sys.path.insert(0, str(scripts))
loader = SourceFileLoader('map_localizer_node', str(scripts / 'map_localizer'))
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
loader.exec_module(module)


def stamp(seconds):
    return Time(nanoseconds=round(seconds * 1e9), clock_type=ClockType.ROS_TIME)


def seed(frame='map'):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = frame
    message.pose.pose.orientation.w = 1.0
    return message


def transform(parent, child, seconds, xyz, yaw=0.):
    message = TransformStamped()
    message.header = Header(frame_id=parent, stamp=stamp(seconds).to_msg())
    message.child_frame_id = child
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = xyz
    message.transform.rotation.z = math.sin(yaw / 2.)
    message.transform.rotation.w = math.cos(yaw / 2.)
    return message


def run():
    rclpy.init(args=['--ros-args', '-p', 'scan_frames:=2', '-p', 'validation_frames:=1'])
    node = module.MapLocalizer()
    now = [10.0]
    node.get_clock = lambda: SimpleNamespace(now=lambda: stamp(now[0]))
    node.last_clock = stamp(now[0]).nanoseconds
    module.time = SimpleNamespace(monotonic=lambda: now[0])
    node.ready_pub = Mock()
    node.status_pub = Mock()
    node.broadcaster = Mock()
    node.executor_pool.submit = Mock()
    status = lambda: json.loads(node.status_pub.publish.call_args.args[0].data)

    def observe(seconds, base_x=1.0, with_transform=True):
        now[0] = seconds
        if with_transform:
            base = transform('odom', 'base', seconds, (base_x, 2.0, 0.4), math.pi / 2.)
            node.buffer.set_transform(base, 'fast_lio')
        header = Header(stamp=stamp(seconds).to_msg(), frame_id='odom')
        node.on_cloud(create_cloud_xyz32(header, np.ones((32, 3), dtype=np.float32)))

    try:
        assert node.state == 'WAITING_FOR_INITIAL_POSE'
        node.on_seed(seed())
        assert node.state == 'COLLECTING' and node.seed == [0., 0., 0.32, 0.]
        observe(9.99, with_transform=False)
        assert not node.frames
        observe(10.1, with_transform=False)
        observe(10.2, with_transform=False)
        observe(10.3, with_transform=False)
        node.poll()
        assert not node.job and node.wait_reason == 'missing_timestamped_odom_base_transform'

        observe(10.4)
        observe(10.5)
        observe(10.6)
        node.poll()
        assert not node.job and node.wait_reason == 'missing_timestamped_odom_lidar_transform'
        node.buffer.set_transform_static(transform('base', 'livox_link', 0., (0.3, 0., 0.2)), 'robot_description')
        observe(10.7, base_x=1.6)
        node.poll()
        assert not node.job and node.wait_reason == 'robot_moved_during_collection'

        for seconds in (10.8, 10.9, 11.0):
            observe(seconds)
        first_job = Future()
        node.executor_pool.submit.return_value = first_job
        node.poll()
        assert node.state == 'MATCHING'
        np.testing.assert_allclose(node.executor_pool.submit.call_args.args[-1], [0.3, 0., 0.2], atol=1e-12)
        accepted = dict(success=True, reason='OK', pose=[3., 4., 0.5, math.pi], metrics={}, seconds=0.1)
        node.on_seed(seed(frame='odom'))
        first_job.set_result(accepted)
        node.poll()
        assert node.state == 'FAILED' and status()['reason'] == 'initial_pose_frame_mismatch'
        assert not node.ready_pub.publish.call_args.args[0].data and node.transform is None
        node.broadcaster.sendTransform.assert_not_called()

        node.on_seed(seed())
        for seconds in (11.1, 11.2, 11.3):
            observe(seconds)
        observe(11.4, with_transform=False)
        second_job = Future()
        node.executor_pool.submit.return_value = second_job
        node.poll()
        second_job.set_result(accepted)
        node.poll()
        assert node.state == 'LOCALIZED' and node.ready_pub.publish.call_args.args[0].data
        assert status()['request_id'] == 3 and status()['reference_stamp_ns'] == stamp(11.1).nanoseconds
        assert status()['fix_stamp_ns'] == stamp(11.4).nanoseconds
        assert node.transform.header.frame_id == 'map' and node.transform.child_frame_id == 'odom'
        translation = node.transform.transform.translation
        np.testing.assert_allclose([translation.x, translation.y, translation.z], [5., 3., 0.1])

        observe(14.4, with_transform=False)
        node.poll()
        assert node.state == 'LOST' and status()['reason'] == 'stale_odom_base_transform'
        assert not node.ready_pub.publish.call_args.args[0].data and node.transform is None
        published = node.broadcaster.sendTransform.call_count
        node.on_seed(seed())
        now[0] += 26.0
        node.poll()
        assert node.state == 'FAILED' and status()['reason'] == 'waiting_for_fresh_registered_cloud'
        assert node.broadcaster.sendTransform.call_count == published
        print('PASS: stamped base/lidar transforms, stationary window, supersession, fix, stream loss, timeout')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    run()
