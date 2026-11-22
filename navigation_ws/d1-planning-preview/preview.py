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
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from scan_planner.msg import Bspline
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class TemporarilyBlocked(RuntimeError):
    """The target is valid but current obstacle observations block the route."""


def path_from_map(start, direction, goal=None, obstacles=None):
    with Path('/workspace/maps/real_final/planning.pickle').open('rb') as stream:
        map_data = pickle.load(stream)
    data = np.asarray(map_data['data'], dtype=float)
    if data.shape[1] != 1:
        raise ValueError('This preview expects a single-floor map')
    height = data[3, 0]
    cost = data[0, 0]
    known = np.isfinite(height) & np.isfinite(cost) & (cost > 0) & (cost <= 20)
    origin = np.asarray(map_data['center'])
    resolution = float(map_data['resolution'])
    offset = np.asarray(cost.shape) // 2

    def cell(point):
        return np.rint((point - origin) / resolution).astype(int) + offset

    def free(index):
        return bool(np.all(index >= 0) and np.all(index < cost.shape) and known[tuple(index)])

    start_cell = cell(start)
    if not free(start_cell):
        raise ValueError(f'起点在静态地图可通行区域之外: {start.tolist()}')
    if obstacles is not None and len(obstacles):
        from scipy.spatial import cKDTree
        indices = cell(np.asarray(obstacles))
        inside = ((indices >= 0) & (indices < cost.shape)).all(axis=1)
        indices = indices[inside]
        if len(indices):
            # Measure from each cell center to actual returns. Rounding both
            # ends to cells can move the inferred obstacle by half a diagonal.
            points = np.asarray(obstacles)[inside]
            radius = .48 + resolution * math.sqrt(.5)
            reach = math.ceil(max(.70, radius) / resolution) + 1
            lo = np.maximum(indices.min(axis=0)-reach, 0)
            hi = np.minimum(indices.max(axis=0)+reach+1, cost.shape)
            region = tuple(slice(x,y) for x,y in zip(lo,hi))
            grid = np.indices(tuple(hi-lo)).reshape(2,-1).T + lo
            tree = cKDTree(points)
            separation = tree.query((grid-offset)*resolution + origin)[0].reshape(tuple(hi-lo))
            known[region] &= separation > radius
            cost[region] = np.minimum(20., cost[region] + np.clip((.70-separation)*10., 0., 6.))
            if not free(start_cell):
                dist, nearest = tree.query(start)
                dx, dy = points[nearest] - start
                forward = dx*math.cos(direction) + dy*math.sin(direction)
                left = -dx*math.sin(direction) + dy*math.cos(direction)
                center = (start_cell-offset)*resolution + origin
                cell_distance = tree.query(center)[0]
                raise TemporarilyBlocked(
                    f'起步净空不足：最近回波距机身中心 {dist:.2f} m'
                    f'（{"前" if forward >= 0 else "后"} {abs(forward):.2f} m，'
                    f'{"左" if left >= 0 else "右"} {abs(left):.2f} m）；'
                    f'起点格中心净空 {cell_distance:.2f} m，要求大于 {radius:.2f} m')
    planner = a_star.Astar()
    planner.init(20., 1, resolution, 1., np.where(known, cost, 100.),
                 np.nan_to_num(height, nan=-100.), np.zeros(cost.shape))
    candidates = [np.asarray(goal)] if goal else [start + distance * np.array([math.cos(direction + angle), math.sin(direction + angle)])
                                                   for distance in (1., .8) for angle in (0., .35, -.35, .7, -.7)]
    for endpoint in candidates:
        goal_cell = cell(endpoint)
        if not free(goal_cell) or np.array_equal(start_cell, goal_cell):
            continue
        if not planner.search(np.array([0, *start_cell[::-1]], dtype=np.int32),
                              np.array([0, *goal_cell[::-1]], dtype=np.int32)):
            continue
        cells = np.vstack((start_cell, np.asarray(planner.get_result_matrix(), dtype=int)[:, 1:]))
        if not np.array_equal(cells[-1], goal_cell):
            continue
        if not all(free(index) for index in cells):
            continue
        route = (cells - offset) * resolution + origin
        route[0], route[-1] = start, endpoint
        # Shortcuts may only cross traversable cells, including both sides of
        # a diagonal cell boundary. Preserve the original route otherwise.
        def visible(a, b):
            count = max(2, math.ceil(np.linalg.norm(b-a)/resolution*8)+1)
            line = np.linspace(a, b, count)
            for dx, dy in ((-1,-1),(-1,1),(1,-1),(1,1)):
                hits = cell(line + np.array([dx,dy])*resolution*.5)
                if np.any(hits < 0) or np.any(hits >= cost.shape) or not known[tuple(hits.T)].all():
                    return False
            return True

        reduced = [route[0]]
        index = 0
        while index < len(route)-1:
            # Retain adjacent A* steps when a longer shortcut grazes a boundary.
            end = len(route)-1
            while end > index+1 and not visible(route[index], route[end]):
                end -= 1
            reduced.append(route[end])
            index = end
        # Pull each corner toward the chord only while BOTH adjacent segments
        # remain in the same inflated free space. Keep a fixed iteration bound.
        for _ in range(8):
            changed = False
            for i in range(1, len(reduced)-1):
                before, corner, after = reduced[i-1:i+2]
                chord = after-before
                fraction = np.clip(np.dot(corner-before, chord) / max(1.e-9, chord@chord), 0., 1.)
                target = before + fraction*chord
                for weight in (.5, .25, .125):
                    trial = corner + weight*(target-corner)
                    if np.linalg.norm(trial-corner) < .015:
                        break
                    if visible(before, trial) and visible(trial, after):
                        reduced[i] = trial
                        changed = True
                        break
            if not changed:
                break
        return np.asarray(reduced)
    if obstacles is not None and len(obstacles):
        path_from_map(start, direction, goal)  # Distinguish a disconnected static map.
        raise TemporarilyBlocked('实时障碍暂时阻断参考路线')
    raise ValueError('静态规划地图上没有可用路线')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'))
    options = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('d1_planning_preview')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(RosPath, '/dryrun/initial_path',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    ready, splines = [], []
    node.create_subscription(Bool, '/localization/ready', lambda msg: ready.append(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def collect_trajectory(message):
        if message.pos_pts and any(math.dist((point.x, point.y, point.z),
                                             (message.pos_pts[0].x, message.pos_pts[0].y, message.pos_pts[0].z)) > .05
                                   for point in message.pos_pts):
            splines.append(message)

    node.create_subscription(Bspline, '/dryrun/bspline', collect_trajectory, 10)
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
