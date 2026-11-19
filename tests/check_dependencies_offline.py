"""Import native modules and verify the bundled PCT cross-floor route."""
import sys
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_share_directory
root = Path(get_package_share_directory('pct_planner'))/'planner'
sys.path[:0] = [str(root), str(root/'scripts')]
from lib import a_star, ele_planner, traj_opt, py_map_manager
import open3d
import cupy
from config import Config
from planner_wrapper import TomogramPlanner

planner = TomogramPlanner(Config())
planner.loadTomogram('building2_9')
start = np.array([-5.5, 6.0, 0.5], dtype=np.float32)
goal = np.array([2.0, -3.0, 4.5], dtype=np.float32)
route = planner.plan(start, goal)
assert route is not None and 20 <= len(route) <= 200
assert np.linalg.norm(route[0, :2] - start[:2]) < 0.2
assert np.array_equal(route[-1, :2], goal[:2])
assert route[-1, 2] - route[0, 2] > 3.5
assert np.array_equal(planner.pos2idx(route[-2, :2]), planner.pos2idx(goal[:2]))
assert route[-2, 2] == route[-1, 2]

center = np.array([*route[-2, :2], goal[2]])
center_route = planner.plan(start, center)
assert len(center_route) == len(route) - 1 and np.array_equal(center_route, route[:-1])
stationary = planner.plan(center, goal, allow_stationary=True)
assert np.array_equal(stationary[-1], route[-1])
assert planner.plan(start, np.array([*goal[:2], 100.0])) is None
assert planner.plan(start, np.array([100.0, 100.0, 0.0])) is None
assert planner.plan(start, np.array([0.0, 0.0, 0.06])) is None
assert np.array_equal(planner.plan(start, goal), route)
print(f'PASS PCT native imports and {len(route)}-point cross-floor route with exact goal XY, '
      f'Open3D {open3d.__version__}, CuPy {cupy.__version__}')
