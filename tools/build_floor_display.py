"""Generate the web console's bundled building2_9 structure sheet from the installed map."""
import base64
import hashlib
import io
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import label
from ament_index_python.packages import get_package_share_directory

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'src/legbot_bringup/scripts'))
from floor_map import FloorMap
from map_match_core import read_pcd


def main():
    atlas = json.loads(FloorMap().payload)
    package = Path(get_package_share_directory('pct_planner'))
    points = read_pcd(package / 'src/pcd/building2_9.pcd')
    points = points[np.isfinite(points).all(axis=1)]
    width, height = atlas['shape']
    indices = np.rint((points[:, :2] - atlas['origin']) / atlas['resolution']).astype(int)
    inside = ((indices >= 0) & (indices < [width, height])).all(axis=1)
    indices = indices[inside]
    flat = indices[:, 0] * height + indices[:, 1]
    elevation = points[inside, 2]
    tiles = []
    checksums = []
    for floor in atlas['floors']:
        mask = base64.b64decode(floor['pixels'])
        checksums.append(hashlib.sha256(mask).hexdigest())
        coverage = np.frombuffer(mask, dtype=np.uint8).reshape(width, height) > 0
        votes = np.zeros(width * height, dtype=np.uint8)
        for lower in (0.30, 0.52, 0.74, 0.96):
            bottom = floor['height'] + lower
            hits = flat[(elevation >= bottom) & (elevation < bottom + 0.22)]
            votes += np.bincount(hits, minlength=width * height) > 0
        tile = np.zeros((width, height), dtype=np.uint8)
        tile[coverage] = 1
        bands = votes.reshape(width, height)
        components, _ = label(bands >= 3, structure=np.ones((3, 3)))
        sizes = np.bincount(components.ravel())
        tile[bands >= 2] = 3
        tile[(components > 0) & (sizes[components] >= 10)] = 2
        tiles.append(tile.T[::-1])
    sheet = Image.fromarray(np.vstack(tiles)).convert('P')
    sheet.putpalette([231, 238, 235, 249, 252, 248, 62, 89, 84, 151, 169, 160] + [0] * (768-12))
    data = io.BytesIO()
    sheet.save(data, format='PNG', bits=2, optimize=True)
    bundle = dict(origin=atlas['origin'], resolution=atlas['resolution'], checksums=checksums,
                  heights=[floor['height'] for floor in atlas['floors']],
                  stairs=[dict(floor=0, to=1, x=-5.25, width=2.1, start=5.65, end=-3.12, count=10),
                          dict(floor=1, to=2, x=8.80, width=2.4, start=-3.45, end=6.0, count=11),
                          dict(floor=2, to=3, x=8.80, width=2.4, start=-3.52, end=5.16, count=10)],
                  png=base64.b64encode(data.getvalue()).decode())
    output = root / 'src/legbot_bringup/web_maps/building2_9.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(bundle, separators=(',', ':')) + '\n')


if __name__ == '__main__':
    main()
