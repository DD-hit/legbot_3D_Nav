"""Validate the generated Building bundle against independent simulator geometry."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root/'tools'))
from build_entry_map import collision_scene
from build_floor_display import FloorMap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    args = parser.parse_args()
    bundle = args.bundle.resolve()
    report = json.loads((bundle/'report.json').read_text())
    assert hashlib.sha256((bundle/'source.pcd').read_bytes()).hexdigest() == report['source_sha256']
    atlas = FloorMap(str(bundle/'planning.pickle'))
    baseline = FloorMap('building2_9.pickle')
    np.testing.assert_array_equal(atlas.center, baseline.center)
    scene = collision_scene(root/'src/legbot_bringup/models/Building.dae')
    offset = np.linspace(-.2, .2, 5)
    probes = np.array([[dx, dy, .035] for dx in offset for dy in offset])

    def check_footprints(xyz):
        for start in range(0, len(xyz), 1000):
            positions = xyz[start:start+1000, None, :] + probes
            rays = np.zeros((*positions.shape[:2], 6), dtype='f4')
            rays[:, :, :3] = positions
            rays[:, :, 5] = -1
            down = scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()
            assert np.all(abs(down-.035) <= .025), 'missing support'
            rays[:, :, 5] = 1
            up = scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()
            assert np.all(up >= .5), 'insufficient overhead clearance'

    assert report['inferred']
    candidates = np.array(report['isolated_candidates'])
    check_footprints(candidates)
    changes = []
    for (heights, allowed), (_, old) in zip(atlas.surfaces, baseline.surfaces):
        assert not np.any(old & ~allowed)
        indices = np.argwhere(allowed & ~old)
        xy = (indices-atlas.offset)*atlas.resolution+atlas.center
        check_footprints(np.column_stack((xy, heights[tuple(indices.T)])))
        changes.append(len(indices))
    spot = atlas.point(dict(x=-.15, y=-3.89, floor=3), True)
    assert abs(spot[2]-4.53)<.02
    stairs = json.loads((bundle/'display.json').read_text())['stairs']
    assert [(s['floor'],s['to']) for s in stairs] == [(0,1),(1,2),(2,3)]
    print(json.dumps(dict(version=report['version'], footprint_samples=25, inferred_cells=len(candidates),
                          new_selectable_by_floor=changes, confirmed_upper_spot=spot, flights=len(stairs))))


if __name__ == '__main__':
    main()
