"""Exercise cancellation and localization loss while a stalled leg is being replanned."""

import sys
import time
import json
import queue
import threading
from pathlib import Path
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/legbot_bringup/scripts'))
from waypoint_tasks import WaypointTasks
from floor_map import FloorMap
from navigation_web import NavigationWeb


def check():
    rclpy.init()
    node = Node('waypoint_task_test')
    try:
        node.ready = True
        node.pct_frame_id, node.base_frame = 'survey_map', 'test_base'
        node.web = None
        node.fix_stamp_ns = 1
        node.initial_odom = (0., 0., 0.4, 0.)
        node.odom_stamp_ns = 1_000_000_000
        node.complete_pub = Mock()
        node.publish_permit = Mock()
        node.plan_ready_pub = Mock()
        node.cancel_localized_mission = Mock()
        tasks = WaypointTasks(node, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.get_clock = lambda: Mock(now=lambda: Time(nanoseconds=node.odom_stamp_ns))
        handle = Mock()
        handle.goal_id.uuid = bytes(range(16))
        handle.request.poses = [PoseStamped(), PoseStamped()]
        handle.is_cancel_requested = False
        tasks.accept(handle)
        tasks.reached()
        assert tasks.state['index'] == 1 and not node.complete_pub.publish.call_args.args[0].data
        node.odom_stamp_ns += 800_000_000
        tasks.tick()
        assert tasks.state['phase'] == 'planning'
        node.started = True
        tasks.progress_time = time.monotonic() - 3
        tasks.tick()
        assert '暂未检测到' in tasks.state['reason'] and tasks.state['phase'] == 'planning'
        tasks.progress_time = time.monotonic() - 7
        tasks.tick()
        assert tasks.stop_terminal == 'replan'
        tasks.tick()
        tasks.tick()
        assert tasks.state['phase'] == 'stopping'
        node.odom_stamp_ns += 700_000_000
        node.fix_stamp_ns = None
        tasks.tick()
        assert tasks.state['phase'] == 'stopping' and tasks.state['index'] == 1
        node.fix_stamp_ns = 2
        tasks.tick()
        assert tasks.replanned and tasks.state['index'] == 1
        tasks.stop('failed', '定位已失效')
        tasks.tick()
        node.odom_stamp_ns += 700_000_000
        tasks.tick()
        assert not tasks.active and tasks.state['phase'] == 'failed'
        assert list(tasks.result.result().missed_waypoints) == [1]

        tasks.accept(handle)
        tasks.stop('replan', '等待重新规划')
        handle.is_cancel_requested = True
        tasks.tick()
        node.odom_stamp_ns += 700_000_000
        tasks.tick()
        assert not tasks.active and tasks.state['phase'] == 'canceled' and tasks.state['index'] == 0
        handle.canceled.assert_called_once()
        print('PASS waypoint stop acknowledgments, fresh odometry, bounded retry, localization loss and cancellation')

        handle.is_cancel_requested = False
        tasks.accept(handle)
        node.tasks = tasks
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.rotation.w = 1.
        transform.transform.translation.x = -30.
        node.buffer = Mock(lookup_transform=Mock(return_value=transform))
        web = NavigationWeb.__new__(NavigationWeb)
        web.mission, web.atlas = node, FloorMap()
        web.localization = {'state': 'LOCALIZED'}
        web.outside_fix = web.seed_pending = web.seed = None
        web.pending, web.condition, web.snapshot = queue.Queue(), threading.Condition(), b''
        web.update()
        assert node.buffer.lookup_transform.call_args.args[:2] == ('survey_map', 'test_base')
        assert tasks.state['phase'] == 'stopping' and json.loads(web.snapshot)['localization']['state'] == 'LOST'
        transform.transform.translation.x = -5.5
        web.update()
        assert not json.loads(web.snapshot)['ready']
        node.fix_stamp_ns += 1
        web.update()
        assert json.loads(web.snapshot)['ready']
        print('PASS map bounds revoke the task and require a new localization fix')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    check()
