#!/usr/bin/python3
"""Authorization and watchdog for body-frame navigation velocity commands."""
import json
import math
import time
import rclpy
from rclpy.executors import ExternalShutdownException
try:
    from rclpy.exceptions import RCLError
except ImportError:  # Jazzy exposes the underlying shutdown error as RuntimeError.
    RCLError = RuntimeError
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Header, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data


class VelocityGuard(Node):
    def __init__(self, node_name, input_topic):
        super().__init__(node_name)
        self.command_timeout = self.declare_parameter('command_timeout', 0.5).value
        self.navigation_timeout = self.declare_parameter('navigation_timeout', 0.5).value
        self.limits = [self.declare_parameter(k, v).value for k, v in
                       [('max_vx', 0.75), ('max_vy', 0.25), ('max_yaw_rate', 1.0)]]
        self.require_navigation_data = self.declare_parameter('require_navigation_data', True).value
        self.require_map_localization = self.declare_parameter('require_map_localization', False).value
        self.fix_stamp_ns = self.permit_stamp_ns = None
        self.last_ready = self.last_permit = float('-inf')
        if self.require_map_localization:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, '/localization/ready', self.on_ready, latched)
            self.create_subscription(String, '/localization/status', self.on_fix, latched)
            self.create_subscription(Header, '/pct/motion_permit', self.on_permit, latched)
        self.last_odom = self.last_cloud = float('-inf')
        self.odom_topic = self.declare_parameter('odom_topic', '/fast_lio/odometry_base').value
        self.cloud_topic = self.declare_parameter('cloud_topic', '/fast_lio/cloud_registered').value
        if self.require_navigation_data:
            self.create_subscription(Odometry, self.odom_topic, self.odom, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, self.cloud_topic, self.cloud, qos_profile_sensor_data)
        self.last_velocity = float('-inf')
        self.velocity = [0., 0., 0.]
        self.last_gate_log = float('-inf')
        self.create_subscription(Twist, input_topic, self.command, 10)
        self.create_timer(0.02, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            'Velocity watchdog inputs: odometry=%s cloud=%s' %
            (self.odom_topic, self.cloud_topic))

    def odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if (msg.header.frame_id and all(math.isfinite(v) for v in values)
                and max(abs(p.x), abs(p.y), abs(p.z)) <= 10000.0
                and 0.5 <= q_norm <= 1.5):
            self.last_odom = time.monotonic()

    def cloud(self, msg):
        if msg.header.frame_id and msg.width * msg.height > 0 and msg.data:
            self.last_cloud = time.monotonic()

    def on_ready(self, msg):
        self.last_ready = time.monotonic() if msg.data else float('-inf')
        if not msg.data:
            self.last_velocity = float('-inf')

    def on_fix(self, msg):
        try:
            status = json.loads(msg.data)
            self.fix_stamp_ns = int(status['fix_stamp_ns']) if status['state'] == 'LOCALIZED' else None
        except (KeyError, ValueError, TypeError):
            self.fix_stamp_ns = None

    def on_permit(self, msg):
        stamp = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec if msg.frame_id else None
        if stamp is None or stamp != self.permit_stamp_ns:
            self.last_velocity = float('-inf')
        self.permit_stamp_ns = stamp
        self.last_permit = time.monotonic()

    def map_permits_motion(self, now):
        return (not self.require_map_localization or
                (self.fix_stamp_ns is not None and self.fix_stamp_ns == self.permit_stamp_ns
                 and now - self.last_ready <= self.navigation_timeout
                 and now - self.last_permit <= self.navigation_timeout))

    def command(self, msg):
        values = [msg.linear.x, msg.linear.y, msg.angular.z]
        if not self.map_permits_motion(time.monotonic()) or not all(math.isfinite(v) for v in values):
            self.velocity = [0., 0., 0.]
            self.last_velocity = float('-inf')
            return
        self.velocity = [max(-limit, min(limit, v)) for v, limit in zip(values, self.limits)]
        self.last_velocity = time.monotonic()

    def allowed_velocity(self, now):
        vx, vy, wz = self.velocity if now-self.last_velocity <= self.command_timeout else (0., 0., 0.)
        if self.require_navigation_data and (
                now-self.last_odom > self.navigation_timeout or
                now-self.last_cloud > self.navigation_timeout):
            if any(abs(value) > 1e-4 for value in (vx, vy, wz)) and now-self.last_gate_log >= 2.0:
                self.last_gate_log = now
                self.get_logger().warning(
                    'Suppressing nonzero command: odometry_fresh=%s cloud_fresh=%s' %
                    (now-self.last_odom <= self.navigation_timeout,
                     now-self.last_cloud <= self.navigation_timeout))
            vx, vy, wz = 0., 0., 0.
            self.last_velocity = float('-inf')
        if not self.map_permits_motion(now):
            self.last_velocity = float('-inf')
            vx, vy, wz = 0., 0., 0.
        return vx, vy, wz


def run(node_type):
    rclpy.init()
    node = node_type()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        try:
            if rclpy.ok():
                node.velocity = [0., 0., 0.]
                node.disarm()
                node.publish()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException, RCLError, RuntimeError):
            # Another launch process can finish the shared shutdown context first.
            pass


