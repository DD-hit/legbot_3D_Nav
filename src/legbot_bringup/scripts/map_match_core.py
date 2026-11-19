"""Gravity-aligned, bounded scan-to-map initialization in building coordinates."""
import math
import os
import time

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree


def read_pcd(path):
    with open(path, 'rb') as stream:
        header = {}
        for raw_line in stream:
            tokens = raw_line.decode('ascii').strip().split()
            if not tokens or tokens[0].startswith('#'):
                continue
            key = tokens[0].upper()
            header[key] = tokens[1:]
            if key == 'DATA':
                break
        names = header['FIELDS']
        counts = list(map(int, header.get('COUNT', ['1'] * len(names))))
        positions = np.cumsum([0] + counts[:-1])
        axes = ['x', 'y', 'z']
        if any(name not in names for name in axes):
            raise ValueError('PCD requires scalar x,y,z fields')
        encoding = header['DATA'][0].lower()
        if encoding == 'ascii':
            return np.loadtxt(stream, usecols=[positions[names.index(name)] for name in axes],
                              ndmin=2, dtype=np.float32)
        if encoding != 'binary':
            raise ValueError('PCD encoding must be ascii or binary')
        formats = {'F': 'f', 'I': 'i', 'U': 'u'}
        fields = []
        for name, kind, size, count in zip(names, header['TYPE'], header['SIZE'], counts):
            field_type = '<' + formats[kind.upper()] + size
            fields.append((name, field_type) if count == 1 else (name, field_type, (count,)))
        expected = int(header['POINTS'][0]) if 'POINTS' in header else -1
        data = np.fromfile(stream, np.dtype(fields), count=expected)
        if expected >= 0 and len(data) != expected:
            raise ValueError('truncated binary PCD')
        return np.column_stack([data[name] for name in axes])


def voxel_points(points, width):
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('point cloud must have shape (n, 3)')
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return points
    cells = np.floor(points / width).astype(np.int64)
    cells -= cells.min(axis=0)
    sizes = cells.max(axis=0) + 1
    keys = (cells[:, 0] * sizes[1] + cells[:, 1]) * sizes[2] + cells[:, 2]
    _, indices = np.unique(keys, return_index=True)
    return points[indices]


def representative_normals(points, tree):
    normals = np.empty_like(points)
    for start in range(0, len(points), 8192):
        part = slice(start, start + 8192)
        indices = tree.query(points[part], k=min(14, len(points)))[1]
        neighbors = points[indices]
        centered = neighbors - neighbors.mean(axis=1, keepdims=True)
        covariance = centered.transpose(0, 2, 1) @ centered
        normals[part] = np.linalg.eigh(covariance)[1][:, :, 0]
    return normals


def subsample(points, count):
    if len(points) <= count:
        return points
    indices = np.random.default_rng(171).choice(len(points), count, replace=False)
    return points[indices]


def scan_points(points, voxel):
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('point cloud must have shape (n, 3)')
    points = points[np.isfinite(points).all(axis=1) & (np.abs(points) < 17.0).all(axis=1)]
    lengths = np.linalg.norm(points, axis=1)
    return voxel_points(points[(lengths > 0.45) & (lengths < 17.0)], voxel).astype(np.float64)


def rotation(yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def angle_delta(first, second):
    return (first - second + np.pi) % (2 * np.pi) - np.pi


def distinct(first, second, distance, angle):
    return (np.linalg.norm(first[:3] - second[:3]) > distance or
            abs(angle_delta(first[3], second[3])) > math.radians(angle))


def observability(normals, lever):
    yaw = normals[:, 1] * lever[:, 0] - normals[:, 0] * lever[:, 1]
    scale = max(1.0, np.sqrt(np.mean(np.sum(lever[:, :2] ** 2, axis=1))))
    jacobian = np.column_stack([normals, yaw / scale])
    eigenvalues = np.linalg.eigvalsh(jacobian.T @ jacobian / len(lever))
    return float(max(0.0, eigenvalues[0]) / max(eigenvalues[-1], 1e-6))


def rejection(metrics):
    if metrics['overlap'] < 0.48:
        return 'LOW_OVERLAP'
    if metrics['rmse'] > 0.175:
        return 'HIGH_RESIDUAL'
    if metrics['geometry'] < 0.012:
        return 'DEGENERATE_GEOMETRY'
    if metrics['structural_support'] < 0.18:
        return 'LOW_STRUCTURAL_SUPPORT'
    if metrics['ray_conflicts'] > 0.15:
        return 'FREE_SPACE_CONFLICT'
    return None


def correlation(grid, source, orientation, shifts):
    weights, lower, resolution = grid
    base = (source @ rotation(orientation).T - lower) / resolution
    cells = np.floor(base[None, :, :] + shifts[:, None, :] / resolution).astype(np.int32)
    sizes = np.array(weights.shape)
    valid = np.all((cells >= 0) & (cells < sizes), axis=2)
    np.clip(cells, 0, sizes - 1, out=cells)
    return (weights[cells[:, :, 0], cells[:, :, 1], cells[:, :, 2]] * valid).mean(axis=1)


class LocalizationMap:
    def __init__(self, path_or_xyz, voxel_size=0.14):
        raw = read_pcd(path_or_xyz) if isinstance(path_or_xyz, (str, os.PathLike)) else path_or_xyz
        self.points = voxel_points(raw, voxel_size).astype(np.float64)
        if len(self.points) < 50:
            raise ValueError('reference map contains too few usable points')
        self.tree = cKDTree(self.points)
        self.normals = representative_normals(self.points, self.tree)
        self.minimum = self.points.min(axis=0)
        self.maximum = self.points.max(axis=0)

    def correlation_grid(self, initial, xy_radius):
        resolution = 0.27
        extent = np.array([18.0 + xy_radius, 18.0 + xy_radius, 14.0])
        lower = np.maximum(self.minimum - 0.6, initial[:3] - extent)
        upper = np.minimum(self.maximum + 0.6, initial[:3] + extent)
        dims = np.ceil((upper - lower) / resolution).astype(int) + 1
        if np.any(dims <= 2):
            return None
        nearby = self.points[np.all((self.points >= lower) & (self.points <= upper), axis=1)]
        if not len(nearby):
            return None
        cells = np.floor((nearby - lower) / resolution).astype(int)
        empty = np.ones(dims, dtype=bool)
        empty[tuple(cells.T)] = False
        distances = distance_transform_edt(empty) * resolution
        weights = np.exp(-0.5 * (distances / 0.38) ** 2).astype(np.float32)
        return weights, lower, resolution

    def coarse_seeds(self, source, initial, xy_radius, yaw_radius, grid):
        source = subsample(voxel_points(source, 0.39), 800)
        offsets = np.linspace(-xy_radius, xy_radius, 2 * math.ceil(xy_radius / 0.50) + 1)
        yaw_offsets = np.linspace(-yaw_radius, yaw_radius, 2 * math.ceil(yaw_radius / 10.0) + 1)
        shifts = np.array([[east, north, 0.0] for east in offsets for north in offsets
                           if math.hypot(east, north) <= xy_radius + 1e-6])
        seeds = []
        for yaw in initial[3] + np.radians(yaw_offsets):
            scores = correlation(grid, source, yaw, initial[:3] + shifts)
            seeds.extend((float(score), np.r_[initial[:3] + shift, yaw]) for score, shift in zip(scores, shifts))
        seeds.sort(key=lambda pair: pair[0], reverse=True)
        selected = []
        for _, pose in seeds:
            if all(distinct(pose, previous, 0.80, 13) for previous in selected):
                selected.append(pose)
            if len(selected) == 10:
                break
        if all(distinct(initial, previous, 0.35, 6) for previous in selected):
            selected.append(initial)
        return selected

    def other_height_seeds(self, source, initial, best_pose, xy_radius, grid):
        source = subsample(voxel_points(source, 0.36), 400)
        offsets = np.linspace(-xy_radius, xy_radius, 2 * math.ceil(xy_radius / 0.75) + 1)
        shifts = [[initial[0] + east, initial[1] + north, 0.0] for east in offsets for north in offsets
                  if math.hypot(east, north) <= xy_radius + 1e-6]
        shifts = np.array(shifts + [best_pose[:3].tolist()])
        lower = max(self.minimum[2] - 0.4, best_pose[2] - 6.0)
        upper = min(self.maximum[2] + 0.5, best_pose[2] + 6.0)
        heights = np.arange(lower, upper + grid[2], grid[2])
        scores = []
        for height in heights[np.abs(heights - best_pose[2]) > 0.80]:
            shifts[:, 2] = height
            likelihood = correlation(grid, source, best_pose[3], shifts)
            for index in np.argsort(likelihood)[::-1][:3]:
                scores.append((likelihood[index], np.r_[shifts[index], best_pose[3]]))
        selected = []
        for _, pose in sorted(scores, key=lambda pair: pair[0], reverse=True):
            same_level = [previous for previous in selected if abs(pose[2] - previous[2]) < 0.70]
            if len(same_level) >= 2 or any(np.linalg.norm(pose[:2] - previous[:2]) < 0.85 for previous in same_level):
                continue
            selected.append(pose)
            if len(selected) == 8:
                break
        return selected

    def refine(self, source, normals, seed, initial, xy_radius, yaw_radius):
        pose = seed.copy()
        for iteration in range(24):
            transform = rotation(pose[3])
            offsets = source @ transform.T
            moved = offsets + pose[:3]
            distance, indices = self.tree.query(moved)
            keep = distance < (0.78 if iteration < 6 else 0.44)
            if np.count_nonzero(keep) < 40:
                return None
            offsets = offsets[keep]
            target_normals = self.normals[indices[keep]]
            source_normals = normals[keep] @ transform.T
            residual = moved[keep] - self.points[indices[keep]]
            cosine = np.sum(source_normals * target_normals, axis=1)
            strength = 0.97
            precision = np.broadcast_to(np.eye(3) * 0.5, (len(residual), 3, 3)).copy()
            for sign in (1, -1):
                basis = source_normals + sign * target_normals
                coefficient = strength / (4 * (2 - strength * (1 + sign * cosine)))
                precision += coefficient[:, None, None] * basis[:, :, None] * basis[:, None, :]
            robust = np.minimum(1.0, 0.18 / np.maximum(distance[keep], 1e-6))
            precision *= robust[:, None, None]
            jacobian = np.zeros((len(offsets), 3, 4))
            jacobian[:, :, :3] = np.eye(3)
            jacobian[:, 0, 3] = -offsets[:, 1]
            jacobian[:, 1, 3] = offsets[:, 0]
            weighted = jacobian.transpose(0, 2, 1) @ precision
            hessian = (weighted @ jacobian).sum(axis=0)
            gradient = (weighted @ residual[:, :, None]).sum(axis=0).ravel()
            increment = np.linalg.solve(hessian + np.eye(4) * 1e-5, -gradient)
            pose += increment
            if (np.linalg.norm(pose[:2] - initial[:2]) > xy_radius + 0.35 or
                    abs(pose[2] - initial[2]) > 0.65 or
                    abs(angle_delta(pose[3], initial[3])) > math.radians(yaw_radius + 5)):
                return None
            if iteration >= 6 and np.linalg.norm(increment) < 0.0015:
                break
        pose[3] = angle_delta(pose[3], 0.0)
        return pose

    def ray_conflicts(self, directions, lengths, origin):
        ranges = np.arange(0.52, lengths.max(), 0.21)
        rows, columns = np.where(ranges[None, :] < lengths[:, None] - 0.50)
        samples = origin + directions[rows] * ranges[columns, None]
        distances, indices = self.tree.query(samples, distance_upper_bound=0.17)
        near = np.isfinite(distances)
        incidence = np.abs((self.normals[indices[near]] * directions[rows[near]]).sum(axis=1))
        evidence = np.exp(-0.5 * (distances[near] / 0.10) ** 2) * np.minimum(1.0, incidence / 0.65)
        blocked_rays = np.unique(rows[near][evidence > 0.55])
        return float(len(blocked_rays) / len(lengths))

    def evaluate(self, source, source_normals, pose, sensor_origin):
        transform = rotation(pose[3]).T
        offsets = source @ transform
        distances, indices = self.tree.query(offsets + pose[:3])
        keep = distances < 0.26
        overlap = float(keep.mean())
        quality = float(np.mean(np.exp(-0.5 * (np.minimum(distances, 2.0) / 0.20) ** 2)))
        metrics = dict(overlap=overlap, quality=quality, rmse=1.0, geometry=0.0,
                       structural_support=0.0, ray_conflicts=1.0, score=0.0)
        if np.count_nonzero(keep) < 30:
            return metrics
        measured_normals = source_normals @ transform
        normals = self.normals[indices]
        aligned = keep & (np.abs((normals * measured_normals).sum(axis=1)) > 0.70)
        structural = np.abs(measured_normals[:, 2]) < 0.60
        support = np.count_nonzero(aligned & structural) / max(1, np.count_nonzero(structural))
        if np.count_nonzero(aligned) >= 30:
            metrics['geometry'] = min(observability(normals[aligned], offsets[aligned]),
                                      observability(measured_normals[aligned], offsets[aligned]))
        origin = sensor_origin @ transform
        rays = offsets - origin
        lengths = np.linalg.norm(rays, axis=1)
        conflicts = self.ray_conflicts(rays / np.maximum(lengths[:, None], 1e-6), lengths,
                                       pose[:3] + origin)
        metrics.update(rmse=float(np.sqrt(np.mean(distances[keep] ** 2))), structural_support=float(support),
                       ray_conflicts=conflicts, score=0.75 * quality + 0.25 * support - 0.65 * conflicts)
        return metrics

    def match(self, scan_xyz, initial_pose, mode='multi', validation_xyz=None,
              xy_radius=2.5, yaw_radius=45.0, sensor_origin=(0.0, 0.0, 0.0)):
        started = time.monotonic()
        initial = np.asarray(initial_pose, dtype=float)
        if (initial.shape != (4,) or not np.all(np.isfinite(initial)) or
                mode not in ('multi', 'single') or not 0 < xy_radius <= 8 or not 0 < yaw_radius <= 180):
            raise ValueError('expected finite x,y,z,yaw, valid mode and bounded search radii')
        sensor_origin = np.asarray(sensor_origin, dtype=float)
        if sensor_origin.shape != (3,) or not np.isfinite(sensor_origin).all() or np.linalg.norm(sensor_origin) > 17:
            raise ValueError('sensor origin must be finite and inside the scan range')
        metrics = {}

        def result(reason, pose=None):
            return dict(success=reason == 'OK', reason=reason, pose=pose.tolist() if pose is not None else None,
                        metrics=metrics, seconds=time.monotonic() - started)

        if (np.any(initial[:2] < self.minimum[:2] - xy_radius) or
                np.any(initial[:2] > self.maximum[:2] + xy_radius) or
                not self.minimum[2] - 0.8 <= initial[2] <= self.maximum[2] + 0.8):
            return result('MAP_OUT_OF_COVERAGE')
        source = scan_points(scan_xyz, 0.17)
        if len(source) < 80:
            return result('INSUFFICIENT_POINTS')
        source = subsample(source, 2500)
        source_normals = representative_normals(source, cKDTree(source))
        grid = self.correlation_grid(initial, xy_radius) if mode == 'multi' else None
        if mode == 'multi' and grid is None:
            return result('MAP_OUT_OF_COVERAGE')
        seeds = self.coarse_seeds(source, initial, xy_radius, yaw_radius, grid) if grid is not None else [initial]
        estimates = []
        for seed in seeds:
            pose = self.refine(source, source_normals, seed, initial, xy_radius, yaw_radius)
            if pose is not None:
                estimates.append((self.evaluate(source, source_normals, pose, sensor_origin), pose))
        metrics['candidate_count'] = len(seeds)
        if not estimates:
            return result('LOW_OVERLAP')
        estimates.sort(key=lambda pair: pair[0]['score'], reverse=True)
        viable = [pair for pair in estimates if not rejection(pair[0])]
        if not viable:
            best = estimates[0][0]
            metrics.update(best)
            return result(rejection(best))
        best, pose = viable[0]
        metrics.update(best)
        for other, candidate in viable[1:]:
            if distinct(pose, candidate, 0.42, 8):
                metrics['candidate_margin'] = best['score'] - other['score']
                if metrics['candidate_margin'] < 0.035:
                    return result('AMBIGUOUS')
                break
        if (np.linalg.norm(pose[:2] - initial[:2]) > xy_radius or
                abs(angle_delta(pose[3], initial[3])) > math.radians(yaw_radius) or
                abs(pose[2] - initial[2]) > 0.50):
            return result('SEARCH_BOUNDARY')
        validation = None
        if validation_xyz is not None:
            validation = subsample(scan_points(validation_xyz, 0.17), 2500)
            if len(validation) < 80:
                return result('INSUFFICIENT_POINTS')
            held_normals = representative_normals(validation, cKDTree(validation))
            held = self.evaluate(validation, held_normals, pose, sensor_origin)
            metrics.update(validation_overlap=held['overlap'], validation_rmse=held['rmse'],
                           validation_ray_conflicts=held['ray_conflicts'],
                           validation_structural_support=held['structural_support'])
            failed = rejection(held)
            if failed:
                metrics['validation_reason'] = failed
                return result('VALIDATION_FAILED')
        if grid is not None:
            other_seeds = self.other_height_seeds(source, initial, pose, xy_radius, grid)
            metrics['other_height_candidates'] = len(other_seeds)
            for seed in other_seeds:
                alternate_initial = initial.copy()
                alternate_initial[2] = seed[2]
                alternate_pose = self.refine(source, source_normals, seed, alternate_initial, xy_radius, yaw_radius)
                if alternate_pose is None or abs(alternate_pose[2] - pose[2]) < 0.80:
                    continue
                alternate = self.evaluate(source, source_normals, alternate_pose, sensor_origin)
                if rejection(alternate) or alternate['score'] < best['score'] - 0.025:
                    continue
                if validation is not None:
                    other_held = self.evaluate(validation, held_normals, alternate_pose, sensor_origin)
                    if rejection(other_held) or other_held['score'] < held['score'] - 0.025:
                        continue
                metrics.update(other_height_m=float(alternate_pose[2]),
                               other_height_score=alternate['score'])
                return result('AMBIGUOUS_FLOOR')
        return result('OK', pose)
