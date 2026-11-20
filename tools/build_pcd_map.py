"""Build a portable PCT and Web map bundle from a gravity-aligned PCD."""
import argparse
import hashlib
import json
import os
import pickle
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml
from scipy.ndimage import binary_closing, convolve, label, maximum_filter, minimum_filter
from scipy.spatial import cKDTree

from build_floor_display import build_display, FloorMap, read_pcd, root
from map_match_core import file_hash, write_reference_index
from floor_surface import reconstruct, reconstruct_tiles

tomography = root / 'src/pct_planner/tomography'
sys.path[:0] = [str(tomography / 'scripts'), str(tomography)]
from config import SceneBuilding
from tomogram import Tomogram


def isolated_samples(points, index, shape, origin, resolution, levels, baseline):
    result = []
    for level in levels:
        costs, heights = baseline[0], baseline[3]
        nearby_heights = heights[(np.abs(heights-level)<.12) & (costs>0) & (costs<=20)]
        if not len(nearby_heights):
            continue
        on_floor = np.abs(points[:, 2] - np.median(nearby_heights)) < .02
        if not on_floor.any():
            continue
        low = np.full(shape, np.inf)
        high = np.full(shape, -np.inf)
        cells = tuple(index[on_floor].T)
        np.minimum.at(low, cells, points[on_floor, 2])
        np.maximum.at(high, cells, points[on_floor, 2])
        missing = ~np.isfinite(low)
        filled = ~missing
        surrounded = convolve(filled.astype(int), np.ones((5, 5), int), mode='constant') == 24
        local_min = minimum_filter(low, size=5)
        local_max = maximum_filter(high, size=5)
        obstructed = np.zeros(shape, bool)
        floor_height = local_max[tuple(index.T)]
        relative = points[:, 2] - floor_height
        obstructed[tuple(index[(relative > .025) & (relative < SceneBuilding.trav.interval_min + .08)].T)] = True
        lip = np.zeros(shape, bool)
        lip[tuple(index[(relative < -.021) & (relative > -.055)].T)] = True
        near_edge = maximum_filter(lip, size=7) | maximum_filter(obstructed, size=5)
        candidates = np.argwhere(missing & surrounded & (local_max - local_min < .025) & ~near_edge)
        samples = points[on_floor]
        tree = cKDTree(samples[:, :2])
        for row, col in candidates:
            xy = origin + np.array([row, col]) * resolution
            nearby = samples[tree.query_ball_point(xy, .95 * resolution)]
            quadrants = (nearby[:, 0] > xy[0]).astype(int) * 2 + (nearby[:, 1] > xy[1])
            if len(np.unique(quadrants)) == 4:
                result.append([*xy, np.median(nearby[:, 2])])
    return np.asarray(result, dtype=np.float32).reshape(-1, 3)


def stair_flights(data, levels):
    cost, ground = data['data'][[0, 3]].astype(float)
    resolution = data['resolution']
    origin = data['center'] - np.array(ground.shape[1:]) // 2 * resolution
    stairs = []
    for floor, (low, high) in enumerate(zip(levels, levels[1:])):
        rise = high - low
        usable = (ground > low + .1 * rise) & (ground < high - .1 * rise) & (cost > 0) & (cost <= 20)
        heights = np.where(usable, ground, -np.inf).max(axis=0)
        areas, count = label(binary_closing(np.isfinite(heights), structure=np.ones((3, 3))), structure=np.ones((3, 3)))
        for region in range(1, count + 1):
            cells = np.argwhere((areas == region) & np.isfinite(heights))
            if len(cells) < 20:
                continue
            z = heights[tuple(cells.T)]
            groups = np.split(np.argsort(z), np.flatnonzero(np.diff(np.sort(z)) > .035) + 1)
            steps = [group for group in groups if len(group) >= max(5, .01 * len(z))]
            if len(steps) < 4:
                continue
            xyz = origin + cells * resolution
            centers = np.array([np.median(xyz[group], axis=0) for group in steps])
            tread_z = np.array([np.median(z[group]) for group in steps])
            if np.ptp(tread_z) < .65 * rise or np.max(np.diff(tread_z)) > SceneBuilding.trav.step_max:
                continue
            direction = np.polyfit(tread_z, centers, 1)[0]
            length = np.linalg.norm(direction)
            if length * rise < 2 * resolution:
                continue
            direction /= length
            if direction[1] < 0:
                direction *= -1
            side = np.array([direction[1], -direction[0]])
            along = centers @ direction
            slope, intercept = np.polyfit(along, tread_z, 1)
            if abs(slope) < .001 or np.max(abs(slope * along + intercept - tread_z)) > .07 or np.ptp(centers @ side) > .35:
                continue
            across = np.concatenate([xyz[group] @ side for group in steps])
            left, right = np.percentile(across, [3, 97])
            stairs.append(dict(floor=floor, to=floor + 1, x=float((left + right) / 2), width=float(right-left+resolution),
                               angle=float(np.arctan2(side[1], side[0])), start=float((low-intercept)/slope),
                               end=float((high-intercept)/slope), count=int(round(rise / np.median(np.diff(tread_z))))))
    return stairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--localization-pcd', type=Path, help='localization reference when planning uses filtered samples')
    parser.add_argument('--profile', type=Path, required=True, help='map id, frame and nominal floor levels/names')
    parser.add_argument('--infer-isolated', action='store_true', help='infer isolated cells on flat platforms from surrounding PCD samples')
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text())
    levels = np.array([item[0] for item in profile['floors']], dtype=float)
    if not len(levels) or not np.isfinite(levels).all() or np.any(np.diff(levels) <= 0):
        parser.error('floor levels must be finite and strictly increasing')
    destination = args.output.resolve()
    if destination.exists():
        parser.error('output must be a new directory')
    points = read_pcd(args.pcd)
    points = points[np.isfinite(points).all(axis=1)].astype('f4')
    if not len(points) or (points[:, 2].max() <= levels[0] and not (len(levels)==1 and 'floor_ranges' in profile)):
        parser.error('PCD needs finite samples above the configured ground elevation')
    minimum, maximum = points.min(axis=0), points.max(axis=0)
    resolution, spacing = SceneBuilding.map.resolution, SceneBuilding.map.slice_dh
    slice_reference = levels[0]
    if profile.get('slice_reference') == 'cloud_min':
        slice_reference = float(np.floor(minimum[2] / spacing) * spacing)
    shape = np.ceil((maximum[:2]-minimum[:2])/resolution).astype(int)+4
    center = (maximum[:2]+minimum[:2])/2
    origin = center - shape//2*resolution
    index = np.rint((points[:, :2]-center)/resolution).astype(int)+shape//2
    bands = profile.get('floor_ranges')
    band_stats = []
    additions = []
    samples = np.empty((0, 3))
    if bands is not None:
        bands = np.asarray(bands, dtype=float)
        if (bands.shape != (len(levels), 2) or not np.isfinite(bands).all() or
                np.any(bands[:, 0] >= bands[:, 1]) or np.any(bands[:, 0] > levels) or
                np.any(bands[:, 1] < levels) or np.any(bands[:-1, 1] >= bands[1:, 0])):
            parser.error('楼层采样范围无效或相互重叠')
        robot = yaml.safe_load((root/'config/robots/go2.yaml').read_text())
        footprint = robot['footprint_radius'] + abs(robot['footprint_offset'])
        surface_builder = reconstruct_tiles if profile.get('tile_platforms') else reconstruct
        for level, band in zip(levels, bands):
            arrays, inferred, stats = surface_builder(points, index, shape, level, band, resolution, footprint)
            band_stats.append(stats)
            if len(levels) > 1:
                additions.append(np.column_stack((origin+np.argwhere(inferred)*resolution, arrays[3, 0][inferred])))
    if bands is None or len(levels) > 1:
        mapping = Tomogram(SceneBuilding)
        mapping.initMappingEnv(center, *shape, int(np.ceil((maximum[2]-slice_reference)/spacing)), slice_reference+spacing)
        planning_points = np.vstack([points, *additions]).astype('f4') if additions else points
        arrays = np.stack(mapping.point2map(planning_points)[:5]).astype('f2')
        samples = isolated_samples(points, index, shape, origin, resolution, levels, arrays)
        if args.infer_isolated and len(samples):
            arrays = np.stack(mapping.point2map(np.vstack((planning_points, samples)))[:5]).astype('f2')
    else:
        slice_reference = levels[0]
    data = dict(resolution=resolution, slice_dh=spacing, slice_h0=slice_reference+spacing, center=center, data=arrays)
    stairs = stair_flights(data, levels)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        bundle = Path(temporary) / 'bundle'
        bundle.mkdir()
        source = bundle/'source.pcd'
        shutil.copyfile(args.localization_pcd or args.pcd, source)
        digest = file_hash(source)
        write_reference_index(source, digest)
        with (bundle/'planning.pickle').open('wb') as stream:
            pickle.dump(data, stream, protocol=4)
        version_hash = hashlib.sha256(arrays.tobytes())
        version_hash.update(json.dumps([profile['floors'], stairs], ensure_ascii=False).encode())
        if bands is not None:
            version_hash.update(bands.tobytes())
        version = 'pcd-' + version_hash.hexdigest()[:12]
        atlas = FloorMap(str(bundle/'planning.pickle'), profile['floors'], profile['id'], version)
        display = build_display(json.loads(atlas.payload), points, stairs)
        (bundle/'display.json').write_text(json.dumps(display, separators=(',', ':')))
        manifest = dict(id=profile['id'], version=version, frame=profile['frame'], floors=profile['floors'],
                        pcd='./source.pcd', tomogram='./planning', display='./display.json')
        if bands is not None:
            manifest['floor_ranges'] = bands.tolist()
        (bundle/'map.yaml').write_text(yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False))
        report = dict(source_sha256=digest, planning_source_sha256=file_hash(args.pcd) if args.localization_pcd else digest, points=len(points),
                      version=version, resolution=resolution, slices=arrays.shape[1], inferred=bool(args.infer_isolated),
                      isolated_candidates=samples.tolist(), stairs=stairs,
                      selectable_cells=[int(allowed.sum()) for _, allowed in atlas.surfaces])
        if bands is not None:
            report.update(floor_ranges=bands.tolist(), floor_samples=band_stats)
        (bundle/'report.json').write_text(json.dumps(report, indent=2))
        os.rename(bundle, destination)
    print(f'{version}: {len(samples)} isolated candidates, {len(stairs)} stair flights, {destination}')


if __name__ == '__main__':
    main()
