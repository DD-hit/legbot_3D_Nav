"""Rebuild the Building web map with isolated, mesh-confirmed entry slab samples."""
import argparse
import json
import os
import pickle
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.ndimage import convolve

root = Path(__file__).resolve().parents[1]
pct = root / 'src/pct_planner'
sys.path[:0] = [str(root / 'src/legbot_bringup/scripts'), str(pct / 'tomography/scripts'), str(pct / 'tomography')]
from config import SceneBuilding
from map_match_core import read_pcd
from tomogram import Tomogram


def collision_scene(path):
    document = ET.parse(path)
    meshes = {}
    for geometry in document.findall('.//{*}library_geometries/{*}geometry'):
        mesh = geometry.find('{*}mesh')
        key = mesh.find('{*}vertices/{*}input[@semantic="POSITION"]').get('source')[1:]
        positions = mesh.findtext(f'{{*}}source[@id="{key}"]/{{*}}float_array')
        vertices = np.fromstring(positions, sep=' ').reshape(-1, 3)
        indices = []
        for part in mesh.findall('{*}triangles'):
            stride = 1 + max(int(item.get('offset')) for item in part.findall('{*}input'))
            offset = int(part.find('{*}input[@semantic="VERTEX"]').get('offset'))
            indices.append(np.fromstring(part.findtext('{*}p'), sep=' ', dtype=np.uint32).reshape(-1, 3, stride)[:, :, offset])
        meshes['#' + geometry.get('id')] = vertices, np.concatenate(indices)
    scene = o3d.t.geometry.RaycastingScene()

    def visit(node, parent):
        transform = parent @ np.fromstring(node.findtext('{*}matrix'), sep=' ').reshape(4, 4)
        instance = node.find('{*}instance_geometry')
        if instance is not None:
            vertices, indices = meshes[instance.get('url')]
            xyz = vertices @ transform[:3, :3].T + transform[:3, 3]
            scene.add_triangles(o3d.core.Tensor(xyz.astype('f4')), o3d.core.Tensor(indices))
        for child in node.findall('{*}node'):
            visit(child, transform)

    for node in document.findall('.//{*}visual_scene/{*}node'):
        visit(node, np.eye(4))
    return scene


def verified_samples(data, mesh):
    ground, ceiling = data['data'][3:5, 0].astype(float)
    resolution = float(data['resolution'])
    origin = data['center'] - (np.array(ground.shape) // 2) * resolution
    flat = np.isfinite(ground) & (np.abs(ground - 0.058) < 0.02)
    around = np.ones((3, 3), int)
    around[1, 1] = 0
    isolated = ~np.isfinite(ground) & (convolve(flat.astype(int), around, mode='constant') == 8)
    samples = []
    for row, col in np.argwhere(isolated):
        neighborhood = ground[row-1:row+2, col-1:col+2]
        height = np.nanmedian(neighborhood)
        if np.nanmax(neighborhood) - np.nanmin(neighborhood) > .02:
            continue
        if not np.all(ceiling[row-1:row+2, col-1:col+2] - height >= SceneBuilding.trav.interval_min + .05):
            continue
        xy = origin + np.array([row, col]) * resolution
        samples.append([*xy, height])
    samples = np.asarray(samples).reshape(-1, 3)
    if not len(samples):
        return samples, int(isolated.sum())
    offsets = np.arange(-2, 3) * resolution
    footprint = np.array([[dx, dy, .04] for dx in offsets for dy in offsets])
    probes = samples[:, None, :] + footprint
    directions = np.zeros_like(probes)
    directions[:, :, 2] = 1

    def cast(vectors):
        rays = np.concatenate((probes, vectors), axis=-1).astype('f4')
        return mesh.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()

    supported = np.all(np.abs(cast(-directions) - .04) < .015, axis=1)
    clear = np.all(cast(directions) >= SceneBuilding.trav.interval_min + .05, axis=1)
    for level in (.11, .26, .41, .55):
        points = probes.copy()
        points[:, :, 2] = samples[:, None, 2] + level
        distance = mesh.compute_distance(o3d.core.Tensor(points.astype('f4'))).numpy()
        clear &= np.all(distance > .095, axis=1)
    return samples[supported & clear], int(isolated.sum())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    directory = pct / 'src/tomogram'
    with (directory / 'building2_9.pickle').open('rb') as stream:
        original = pickle.load(stream)
    cloud = read_pcd(pct / 'src/pcd/building2_9.pcd')
    mesh = collision_scene(root / 'src/legbot_bringup/models/Building.dae')
    samples, candidates = verified_samples(original, mesh)
    slices = int(np.ceil(cloud[:, 2].max() / original['slice_dh']))
    generator = Tomogram(SceneBuilding)
    generator.initMappingEnv(original['center'], *original['data'].shape[-2:], slices, original['slice_h0'])

    def rebuild(points):
        return np.stack(generator.point2map(np.ascontiguousarray(points, dtype='f4'))[:5]).astype('f2')

    if not np.array_equal(rebuild(cloud), original['data'], equal_nan=True):
        raise ValueError('Bundled baseline differs from the current Building profile')
    modified = rebuild(np.vstack((cloud, samples)))
    if modified.shape != original['data'].shape or not len(samples):
        raise ValueError('Unexpected layers or no confirmed repairs')
    result = dict(original, data=modified)
    with tempfile.NamedTemporaryFile(dir=directory, delete=False) as stream:
        pickle.dump(result, stream, protocol=4)
    os.chmod(stream.name, 0o644)
    os.replace(stream.name, directory / 'building2_9_entry.pickle')
    report = dict(candidates=candidates, repaired=len(samples), coordinates=samples.tolist())
    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(f'{len(samples)} of {candidates} isolated entry cells repaired')


if __name__ == '__main__':
    main()
