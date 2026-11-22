import os
import sys
import pickle
from pathlib import Path
import numpy as np
from scipy.interpolate import griddata

from utils import *

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class TomogramPlanner(object):
    ASTAR_COST_THRESHOLD = 20.0

    def __init__(self, cfg):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate

        self.tomo_dir = Path(
            os.environ.get('LEGBOT_DATA_DIR', os.path.join(os.getcwd(), 'data'))
        ) / 'tomogram'
        self.bundled_tomo_dir = Path(__file__).resolve().parents[2] / 'src' / 'tomogram'

        self.tomogram=None
        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.layer_elev_grids = None  # 存储每个layer的高度网格（elev_g）
        self.grid_xy_coords = None    # 存储网格对应的物理XY坐标

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

    def loadTomogram(self, tomo_file):
        candidates = [
            self.tomo_dir / (tomo_file + '.pickle'),
            self.bundled_tomo_dir / (tomo_file + '.pickle'),
        ]
        tomo_path = next((path for path in candidates if path.is_file()), None)
        if tomo_path is None:
            raise FileNotFoundError(
                'tomogram %s not found; checked: %s' %
                (tomo_file, ', '.join(str(path) for path in candidates)))
        print('Loading tomogram:', tomo_path, flush=True)
        with tomo_path.open('rb') as handle:
            data_dict = pickle.load(handle)

            self.tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = self.tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [self.tomogram.shape[2], self.tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = self.tomogram[0]
        trav_gx = self.tomogram[1]
        trav_gy = self.tomogram[2]
        elev_g = self.tomogram[3]
        elev_g = np.nan_to_num(elev_g, nan=-100)
        elev_c = self.tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)
        # 存储所有layer的高度网格（关键：后续用于XY查高度）
        self.layer_elev_grids = elev_g
        # 预计算网格对应的物理XY坐标（避免重复计算）,可行性分析的trav导入，在目标点decide_layer，选择最低cost的layer
        self._precompute_grid_xy()
        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
      


    def _precompute_grid_xy(self):
        """预计算每个网格点对应的物理XY坐标（匹配map的center和resolution）"""
        # 生成网格索引
        grid_h, grid_w = self.map_dim[0], self.map_dim[1]
        y_grid_idx, x_grid_idx = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
        # 转换为物理坐标（反向复用pos2idx的逻辑）
        x_grid = (x_grid_idx - self.offset[0]) * self.resolution + self.center[0]
        y_grid = (y_grid_idx - self.offset[1]) * self.resolution + self.center[1]
        # 存储为 [grid_h, grid_w, 2] （x,y）
        self.grid_xy_coords = np.stack([x_grid, y_grid], axis=-1)

    def get_layer_height_by_xy(self, layer_idx, x, y):
        """
        输入layer索引和物理XY坐标，返回该layer下此XY对应的真实高度
        :param layer_idx: 分层索引（0~n_slice-1）
        :param x: 物理X坐标
        :param y: 物理Y坐标
        :return: 插值后的高度值（无有效值返回-100）
        """
        # 边界检查
        if layer_idx < 0 or layer_idx >= self.n_slice:
            return -100.0

        elev_grid = self.layer_elev_grids[layer_idx]
        xy_coords = self.grid_xy_coords.reshape(-1, 2)  
        elev_vals = elev_grid.reshape(-1)              

        valid_mask = elev_vals > -99.0
        valid_xy = xy_coords[valid_mask]
        valid_elev = elev_vals[valid_mask]

        if len(valid_xy) == 0:
            return -100.0

        query_height = griddata(valid_xy, valid_elev, (x, y), method='nearest', fill_value=-100.0)
        return float(query_height)

    def match_best_layer(self, x, y, target_height):
        """Select the lowest-cost surface at the requested cell and height."""
        column, row = self.pos2idx(np.array([x, y])).astype(int)
        if not (0 <= row < self.map_dim[0] and 0 <= column < self.map_dim[1]):
            return None
        heights = self.layer_elev_grids[:, row, column]
        costs = self.tomogram[0, :, row, column]
        candidates = np.flatnonzero((heights > -99.0) & (np.abs(heights - target_height) <= self.slice_dh)
                                    & np.isfinite(costs) & (costs > 0.0))
        if not len(candidates):
            return None
        return int(candidates[np.argmin(costs[candidates])])
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        self.planner.init_map(
            self.ASTAR_COST_THRESHOLD, 15, self.resolution, self.n_slice, 1,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
        )



    @staticmethod
    def _simplify_route(points, epsilon):
        """Ramer-Douglas-Peucker simplification in XYZ."""
        if len(points) < 3:
            return points
        first = points[0]
        last = points[-1]
        segment = last - first
        denominator = float(segment @ segment)
        middle = points[1:-1]
        if denominator < 1e-12:
            distances = np.linalg.norm(middle - first, axis=1)
        else:
            projection = np.clip(((middle - first) @ segment) / denominator, 0.0, 1.0)
            distances = np.linalg.norm(
                middle - (first + projection[:, None] * segment), axis=1)
        split = int(np.argmax(distances))
        if float(distances[split]) <= epsilon:
            return points[[0, -1]]
        split += 1
        left = TomogramPlanner._simplify_route(points[:split + 1], epsilon)
        right = TomogramPlanner._simplify_route(points[split:], epsilon)
        return np.vstack((left[:-1], right))

    def _astar_route_to_map(self, path):
        """Convert [layer,row,column] A* cells to local Building XYZ."""
        cells = np.rint(np.asarray(path)).astype(np.int32)
        layers = np.clip(cells[:, 0], 0, self.n_slice - 1)
        rows = np.clip(cells[:, 1], 0, self.map_dim[0] - 1)
        columns = np.clip(cells[:, 2], 0, self.map_dim[1] - 1)
        x = (rows - self.map_dim[0] // 2) * self.resolution + self.center[0]
        y = (columns - self.map_dim[1] // 2) * self.resolution + self.center[1]
        # Tomogram elevation is terrain height. PCT's GO2 reference is the
        # walking-base path, 0.5 m above that surface.
        z = self.layer_elev_grids[layers, rows, columns] + 0.5
        route = np.column_stack((x, y, z)).astype(np.float64)
        if not np.isfinite(route).all():
            raise RuntimeError('PCT A* route contains invalid elevation samples')
        epsilon = max(0.0, float(getattr(
            self.cfg.planner, 'route_simplify_epsilon', 0.08)))
        return self._simplify_route(route, epsilon)

    def _finish_at_goal(self, route, end_pos):
        """Extend only within the A*-validated goal cell, at walking-base height."""
        layer, column, row = self.end_idx
        goal_z = self.layer_elev_grids[layer, row, column] + 0.5
        last = route[-1]
        if (not np.isfinite(last).all() or not np.array_equal(self.pos2idx(last[:2]), self.end_idx[1:])
                or abs(last[2] - goal_z) > self.resolution):
            return None
        if np.linalg.norm(last[:2] - end_pos[:2]) <= 1e-6:
            last[:2] = end_pos[:2]
            return route
        return np.vstack((route, [end_pos[0], end_pos[1], last[2]]))

    def plan(self, start_pos, end_pos, optimize=None, allow_stationary=False):
        start_layer = self.match_best_layer(*start_pos)
        end_layer = self.match_best_layer(*end_pos)
        if start_layer is None or end_layer is None:
            return None

        self.start_idx[1:] = self.pos2idx(start_pos[:2])
        self.end_idx[1:] = self.pos2idx(end_pos[:2])
        self.start_idx[0] = start_layer
        self.end_idx[0] = end_layer

        layer, column, row = self.end_idx
        if self.tomogram[0, layer, row, column] > self.ASTAR_COST_THRESHOLD:
            return None
        if np.array_equal(self.start_idx, self.end_idx):
            if not allow_stationary or abs(start_pos[2] - end_pos[2]) > 0.5:
                return None
            goal_z = self.layer_elev_grids[layer, row, column] + 0.5
            return np.vstack((start_pos, [end_pos[0], end_pos[1], goal_z]))

        if optimize is None:
            optimize = bool(getattr(self.cfg.planner, 'optimize_path', False))
        planned = self.planner.plan(self.start_idx, self.end_idx, bool(optimize))
        if not planned:
            return None
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None
        if not optimize:
            return self._finish_at_goal(self._astar_route_to_map(path), end_pos)

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()
        # print("heights origin:", heights)


        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        # print(traj_raw,"traj_raw")
        return traj_3d
    

    def pos2idx(self, pos):
        pos = pos - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        idx = np.array([idx[1], idx[0]], dtype=np.float32)
        return idx
