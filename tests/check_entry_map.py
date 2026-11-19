"""Check the installed web map against the original atlas and the Building collision mesh."""
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'tools'))
from build_entry_map import collision_scene
from floor_map import FloorMap


def main():
    before = FloorMap('building2_9.pickle')
    after = FloorMap()
    assert before.resolution == after.resolution and np.array_equal(before.center, after.center)
    for old, new in zip(before.surfaces[1:], after.surfaces[1:]):
        assert np.array_equal(old[0], new[0], equal_nan=True) and np.array_equal(old[1], new[1])
    old_height, old_allowed = before.surfaces[0]
    height, allowed = after.surfaces[0]
    assert np.all(allowed[old_allowed])
    old_valid = np.isfinite(old_height)
    assert np.array_equal(height[old_valid], old_height[old_valid])
    rows, cols = np.nonzero(allowed & ~old_allowed)
    assert len(rows) and new_target_was_repaired(before, after)
    xy = (np.column_stack((rows, cols)) - after.offset) * after.resolution + after.center
    positions = np.column_stack((xy, height[rows, cols] + .03))
    directions = np.zeros_like(positions)
    directions[:, 2] = 1
    scene = collision_scene(root / 'src/legbot_bringup/models/Building.dae')

    def distances(sign):
        rays = np.column_stack((positions, sign * directions)).astype('f4')
        return scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()

    assert np.all(np.abs(distances(-1) - .03) < .03)
    assert np.all(distances(1) + .03 >= .50)
    print(f'PASS {len(rows)} newly selectable centers have floor support and clearance; upper masks/heights match')


def new_target_was_repaired(before, after):
    example = dict(x=2.15, y=-.89, floor=0)
    try:
        before.point(example, True)
    except ValueError:
        return abs(after.point(example, True)[2] - .058) < .01
    return False


if __name__ == '__main__':
    main()
