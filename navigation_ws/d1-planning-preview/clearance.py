"""Last-command clearance check for the D1 single-floor navigation profile."""
import math
from collections import deque

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.interpolate import BSpline
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points


class BodyBlocked(RuntimeError):
    pass


class BodyClearance:
    # Fixed standing model plus 5 cm padding, relative to mechanical base_link.
    lower = np.array([-.39221, -.23815])
    upper = np.array([.40711, .23817])

    def __init__(self, node):
        self.clock = node.get_clock()
        self.scans = deque(maxlen=12)
        self.pose = None
        self.map_first = self.map_last = None
        node.create_subscription(Header, '/grid_map/update', self.on_map, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/fast_lio/odometry_base', self.on_pose, qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/fast_lio/cloud_registered', self.on_cloud, qos_profile_sensor_data)

    def on_pose(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id not in ('', 'base_link'):
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        origin = np.array([p.x, p.y, p.z])
        if np.isfinite(origin).all() and np.isfinite(quat).all() and np.linalg.norm(quat) > .9:
            self.pose = (Time.from_msg(msg.header.stamp).nanoseconds, origin, Rotation.from_quat(quat))

    def on_cloud(self, msg):
        if msg.header.frame_id != 'odom' or self.pose is None:
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        pose_stamp, origin, _ = self.pose
        if abs(stamp - pose_stamp) > 200_000_000:
            return
        data = read_points(msg, field_names=['x', 'y', 'z'])
        points = np.column_stack([data[k].ravel() for k in ('x', 'y', 'z')])
        points = points[np.isfinite(points).all(axis=1)]
        if len(points):
            self.scans.append((stamp, points[(np.abs(points-origin)<1.3).all(axis=1)]))

    def on_map(self, msg):
        if msg.frame_id != 'odom':
            return
        now = Time.from_msg(msg.stamp).nanoseconds
        if self.map_last is None or now - self.map_last > 1_000_000_000:
            self.map_first = now
        self.map_last = now

    def ready(self):
        now = self.clock.now().nanoseconds
        fresh = {stamp for stamp, _ in self.scans if 0 <= now-stamp < 800_000_000}
        return (len(fresh) >= 5 and self.map_last is not None
                and self.map_last - self.map_first >= 150_000_000
                and 0 <= now - self.map_last < 1_000_000_000
                and 0 <= self.clock.now().nanoseconds - self.scans[-1][0] < 350_000_000)

    def observed_obstacles(self):
        now = self.clock.now().nanoseconds
        _, origin, rotation = self.pose
        batches = [points for stamp, points in self.scans if 0 <= now-stamp < 800_000_000]
        if not batches:
            raise RuntimeError('没有新鲜的障碍观测，停止导航')
        cloud = np.concatenate(batches)
        height = rotation.inv().apply(cloud-origin)[:,2]
        return cloud[(height > -.20) & (height < .49)]

    def path_clearance(self, spline, age=0.):
        if not self.ready():
            return None
        obstacles = self.observed_obstacles()[:, :2]
        if not len(obstacles):
            return math.inf
        knots, degree = spline.knots, spline.order
        a, b = knots[degree], knots[-degree-1]
        a = min(b, a + max(0., age))
        b = min(b, a + 1.2)  # Recheck the upcoming motion beyond the command watchdog.
        curve = BSpline(knots, [[p.x,p.y] for p in spline.pos_pts], degree)
        samples = curve(np.linspace(a, b, max(2, math.ceil((b-a)/.05)+1)))
        distance, _ = cKDTree(obstacles).query(samples)
        return float(distance.min())

    def check(self, cmd):
        if cmd.linear.x == cmd.linear.y == cmd.angular.z == 0.:
            return
        now = self.clock.now().nanoseconds
        if (not self.scans or self.pose is None or
                any(not 0 <= now - stamp < 350_000_000 for stamp in (self.pose[0], self.scans[-1][0]))):
            raise RuntimeError('障碍点云或机身位姿已过期，停止导航')
        _, origin, rotation = self.pose
        # Enclose motion in every direction during the command watchdog window.
        # Include the SDK minimum side step even for a smaller requested vy.
        side = max(.10, abs(cmd.linear.y)) if cmd.linear.y else 0.
        margin = .35 * (math.hypot(cmd.linear.x, side) + .48 * abs(cmd.angular.z))
        points = np.concatenate([p for stamp,p in self.scans if 0 <= now-stamp < 800_000_000])
        body = rotation.inv().apply(points-origin)
        relevant = body[(body[:,2] > -.20) & (body[:,2] < .49), :2]
        if np.any(((relevant >= self.lower-margin) & (relevant <= self.upper+margin)).all(axis=1)):
            raise BodyBlocked('实时障碍进入机身停车范围')
