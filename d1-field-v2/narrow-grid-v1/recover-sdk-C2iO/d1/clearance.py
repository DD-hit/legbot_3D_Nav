"""Last-command clearance check for the D1 single-floor navigation profile."""
import math
from collections import deque

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.interpolate import BSpline
from pathlib import Path
import yaml
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points


class BodyBlocked(RuntimeError):
    pass


class StaticBoundary(RuntimeError):
    def __init__(self):
        super().__init__('局部绕行轨迹越出静态地图可通行区，已停车')


class BodyClearance:
    def __init__(self, node, forward_max=.12, yaw_max=.25, lateral_max=.12):
        self.forward_max, self.yaw_max = forward_max, yaw_max
        config = yaml.safe_load(Path(__file__).with_name('planner.yaml').read_text())
        params = config['/d1_community/scan_planner_node']['ros__parameters']
        half_width = params['grid_map.footprint_half_width']
        self.lower = np.array([-params['grid_map.footprint_rear'], -half_width])
        self.upper = np.array([params['grid_map.footprint_front'], half_width])
        self.radius = np.linalg.norm(np.maximum(-self.lower, self.upper))
        self.lateral_max = lateral_max
        self.cloud_range = max(1.3, self.radius + 1.2*math.hypot(forward_max,lateral_max) + .12)
        self.clock = node.get_clock()
        self.scans = deque(maxlen=12)
        self.on_static_surface = None
        self.pose = None
        self.lidar_stamp = None
        node.create_subscription(Odometry, '/fast_lio/odometry_lidar', self.on_lidar_pose, qos_profile_sensor_data)
        self.map_first = self.map_last = None
        node.create_subscription(PointCloud2, '/d1_community/grid_map/occupancy',
                                 lambda m: self.on_map(m.header), qos_profile_sensor_data)
        node.create_subscription(Odometry, '/fast_lio/odometry_base', self.on_pose, qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/fast_lio/cloud_registered', self.on_cloud, qos_profile_sensor_data)

    def on_lidar_pose(self, msg):
        if msg.header.frame_id == "odom":
            self.lidar_stamp = Time.from_msg(msg.header.stamp).nanoseconds

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
            self.scans.append((stamp, points[(np.abs(points-origin)<self.cloud_range).all(axis=1)]))

    def on_map(self, msg):
        if msg.frame_id != 'odom':
            return
        now = Time.from_msg(msg.stamp).nanoseconds
        if self.map_last is None or now - self.map_last > 1_000_000_000:
            self.map_first = now
        self.map_last = now

    def sensors_ready(self):
        now = self.clock.now().nanoseconds
        fresh = {stamp for stamp, _ in self.scans if 0 <= now-stamp < 800_000_000}
        return (self.lidar_stamp is not None and 0 <= now-self.lidar_stamp < 350_000_000
                and len(fresh) >= 5 and 0 <= now-self.scans[-1][0] < 350_000_000)

    def map_ready(self):
        now = self.clock.now().nanoseconds
        return (self.map_last is not None and self.map_last-self.map_first >= 150_000_000
                and 0 <= now-self.map_last < 1_000_000_000)

    def ready(self):
        return self.sensors_ready() and self.map_ready()

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
        knots, degree = spline.knots, spline.order
        a, b = knots[degree], knots[-degree-1]
        a = min(b, a + max(0., age))
        b = min(b, a + 1.2)  # Recheck the upcoming motion beyond the command watchdog.
        curve = BSpline(knots, [[p.x,p.y] for p in spline.pos_pts], degree)
        samples = curve(np.linspace(a, b, max(2, math.ceil((b-a)/.05)+1)))
        if self.on_static_surface and not self.on_static_surface(samples):
            raise StaticBoundary()
        bound = self.radius+.12
        obstacles = obstacles[((obstacles >= samples.min(axis=0)-bound) &
                               (obstacles <= samples.max(axis=0)+bound)).all(axis=1)]
        if not len(obstacles):
            return math.inf
        # Start with the observed heading. Limit its evolution to the SDK yaw rate.
        yaw = self.pose[2].as_euler('xyz')[2]
        step = (b-a)/(len(samples)-1)
        extra = (self.yaw_max*self.radius + max(.18, math.hypot(self.forward_max,self.lateral_max)))*step
        gap = math.inf
        for i, xy in enumerate(samples):
            if i:
                delta = xy-samples[i-1]
                if np.linalg.norm(delta) > 1.e-6:
                    error = math.atan2(delta[1], delta[0])-yaw
                    yaw += np.clip(math.atan2(math.sin(error), math.cos(error)), -self.yaw_max*step, self.yaw_max*step)
            gap = min(gap, self.edge_gap(obstacles, xy, yaw) - extra)
        return gap

    def edge_gap(self, points, xy, yaw):
        nearby = points[(np.abs(points-xy) <= self.radius+.12).all(axis=1)]-xy
        if not len(nearby):
            return math.inf
        c, s = math.cos(yaw), math.sin(yaw)
        local = nearby @ np.array([[c, -s], [s, c]])
        q = np.maximum(self.lower-local, local-self.upper)
        return float((np.linalg.norm(np.maximum(q, 0.), axis=1) + np.minimum(q.max(axis=1), 0.)).min())

    def check(self, cmd):
        if cmd.linear.x == cmd.linear.y == cmd.angular.z == 0.:
            return
        now = self.clock.now().nanoseconds
        if (not self.scans or self.pose is None or
                any(not 0 <= now - stamp < 350_000_000 for stamp in (self.pose[0], self.scans[-1][0]))):
            raise RuntimeError('障碍点云或机身位姿已过期，停止导航')
        _, origin, rotation = self.pose
        points = self.observed_obstacles()
        body = rotation.inv().apply(points-origin)[:,:2]
        vx, vy, w = cmd.linear.x, cmd.linear.y, cmd.angular.z
        bound = self.radius + .35*math.hypot(vx,vy) + .02
        body = body[(np.abs(body) <= bound).all(axis=1)]
        dt = .05
        interpolation_margin = .5*dt*(math.hypot(vx,vy) + self.radius*abs(w))
        centers = []
        for t in np.linspace(0., .35, 8):
            angle = w*t
            if abs(w) < 1.e-8:
                xy = np.array([vx*t, vy*t])
            else:
                sn, cs = math.sin(angle), 1.-math.cos(angle)
                xy = np.array([vx*sn-vy*cs, vx*cs+vy*sn])/w
            centers.append(xy)
            if self.edge_gap(body, xy, angle) <= interpolation_margin:
                raise BodyBlocked('实时障碍进入机身停车范围')

        if self.on_static_surface and (vx or vy):
            offsets = np.column_stack((centers, np.zeros(len(centers))))
            predicted = rotation.apply(offsets)[:, :2] + origin[:2]
            if not self.on_static_surface(predicted):
                raise StaticBoundary()
