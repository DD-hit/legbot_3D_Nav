import base64
import json
import math
import os
import pickle
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory


class FloorMap:
    def __init__(self, filename='building2_9_entry.pickle', floor_definitions=None, name='building2_9', version='entry-v1'):
        source = Path(filename)
        if not source.is_absolute():
            package = Path(get_package_share_directory('pct_planner'))
            data_dir = Path(os.environ.get('LEGBOT_DATA_DIR', 'data'))
            source = package / 'planner/scripts' / data_dir / 'tomogram' / filename
            if not source.is_file():
                source = package / 'src/tomogram' / filename
        with source.open('rb') as stream:
            data = pickle.load(stream)
        costs, heights = np.asarray(data['data'][0], dtype=float), np.asarray(data['data'][3], dtype=float)
        self.resolution = float(data['resolution'])
        self.center = np.asarray(data['center'], dtype=float)
        self.shape = heights.shape[1:]
        self.offset = np.array(self.shape) // 2
        self.surfaces = []
        floors = []
        definitions = floor_definitions or [(0, '入口层'), (1.5, '下层平台'), (3, '中层平台'), (4.5, '上层平台'), (6, '顶层')]
        levels = np.array([item[0] for item in definitions], dtype=float)
        if not np.all(np.isfinite(levels)) or np.any(np.diff(levels) <= 0):
            raise ValueError('Floor elevations must be finite and strictly increasing')
        boundaries = (levels[:-1] + levels[1:]) / 2
        for index, (nominal, label) in enumerate(definitions):
            lower = boundaries[index - 1] if index else nominal - (boundaries[0] - nominal if len(boundaries) else 0.75)
            upper = boundaries[index] if index < len(boundaries) else nominal + (nominal - boundaries[-1] if len(boundaries) else 0.75)
            valid = (heights >= lower) & (heights < upper) & np.isfinite(costs)
            walkable = valid & (costs > 0) & (costs <= 20)
            score = np.where(walkable.any(axis=0), np.where(walkable, costs, np.inf),
                             np.where(valid, np.abs(heights - nominal), np.inf))
            best = np.argmin(score, axis=0)[None, ...]
            selected = np.take_along_axis(heights, best, axis=0)[0]
            selected = np.where(valid.any(axis=0), selected, np.nan)
            allowed = walkable.any(axis=0)
            self.surfaces.append((selected, allowed))
            pixels = np.where(allowed, 2, np.isfinite(selected)).astype('uint8')
            elevation_cm = np.rint(np.nan_to_num(selected, nan=-327.68) * 100).astype('<i2')
            floors.append(dict(name=label, height=nominal, pixels=base64.b64encode(pixels.tobytes()).decode(),
                               elevation_cm=base64.b64encode(elevation_cm.tobytes()).decode()))
        self.payload = json.dumps(dict(name=name, version=version, resolution=self.resolution, shape=self.shape,
                                       origin=(self.center - self.offset * self.resolution).tolist(),
                                       floors=floors), ensure_ascii=False).encode()

    def contains_base(self, xy):
        cell = (np.asarray(xy) - self.center) / self.resolution + self.offset
        return bool(np.all(cell >= -0.5) and np.all(cell < np.asarray(self.shape) - 0.5))

    def point(self, item, require_walkable):
        if not isinstance(item, dict) or type(item.get('floor')) is not int:
            raise ValueError('请选择楼层')
        floor = item['floor']
        if not 0 <= floor < len(self.surfaces):
            raise ValueError('未知楼层')
        if not all(type(item.get(key)) in (int, float) and math.isfinite(item[key]) for key in ('x', 'y')):
            raise ValueError('地图坐标无效')
        xy = np.array([item['x'], item['y']], dtype=float)
        cell = np.rint((xy - self.center) / self.resolution) + self.offset
        if np.any(cell < 0) or np.any(cell >= self.shape):
            raise ValueError('点位在地图外')
        row, column = cell.astype(int)
        heights, allowed = self.surfaces[floor]
        height = heights[row, column]
        if not np.isfinite(height):
            raise ValueError('所选楼层的该处无有效楼面')
        if require_walkable and not allowed[row, column]:
            raise ValueError('该处楼面的规划代价不满足目标点条件')
        return [*xy.tolist(), float(height)]
