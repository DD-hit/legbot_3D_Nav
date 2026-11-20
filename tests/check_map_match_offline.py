"""Exercise initialization, unobservable scans, occluded hits and duplicate rooms."""
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/legbot_bringup/scripts'))
from map_match_core import LocalizationMap, angle_delta, file_hash, read_reference_index, reference_index_path, rotation, write_reference_index


def surface(fixed_axis, coordinate, bounds):
    axes = [axis for axis in range(3) if axis != fixed_axis]
    grid = [np.arange(*bounds[axis], 0.08) for axis in axes]
    first, second = np.meshgrid(*grid, indexing='ij')
    points = np.full((first.size, 3), coordinate)
    points[:, axes[0]] = first.ravel()
    points[:, axes[1]] = second.ravel()
    return points


def room(bounds):
    return np.vstack([surface(axis, edge, bounds) for axis in range(3) for edge in bounds[axis]])


def observation(points, pose, random):
    selected = random.choice(len(points), len(points) // 2, replace=False)
    local = (points[selected] - pose[:3]) @ rotation(pose[3])
    return local + random.normal(0.0, 0.016, local.shape)


def main():
    random = np.random.default_rng(340)
    bounds = [(-2.4, 3.1), (-1.8, 2.8), (0.0, 2.3)]
    walls = room(bounds)
    reference = LocalizationMap(walls)
    with tempfile.TemporaryDirectory() as directory:
        source_file = Path(directory) / 'room.pcd'
        with source_file.open('wb') as stream:
            stream.write(f'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS {len(walls)}\nDATA binary\n'.encode())
            walls.astype('<f4').tofile(stream)
        write_reference_index(source_file, file_hash(source_file))
        cached = read_reference_index(source_file, .14)
        np.testing.assert_array_equal(cached[0], reference.points)
        np.testing.assert_array_equal(cached[1], reference.normals)
        assert read_reference_index(source_file, .2) is None
        with source_file.open('ab') as stream:
            stream.write(b'\n')
        assert read_reference_index(source_file, .14) is None
        reference_index_path(source_file).write_bytes(b'truncated')
        np.testing.assert_array_equal(LocalizationMap(source_file).points, reference.points)
    truth = np.array([-0.12, 0.06, 0.68, math.radians(168.0)])
    source = observation(walls, truth, random)
    validation = observation(walls, truth, random)
    shifted = truth[:3] + [.13, -.11, .08]
    correction = reference.tracking_error(validation, np.zeros(3), shifted, rotation(truth[3]))
    assert correction is not None
    np.testing.assert_allclose(shifted + correction[:3], truth[:3], atol=.035)
    plane = surface(2, 0.0, bounds)
    single_plane = LocalizationMap(plane)
    assert single_plane.tracking_error(plane, np.zeros(3), np.zeros(3), np.eye(3)) is None
    seed = truth + [1.5, -0.50, 0.12, math.radians(38.0)]
    assert reference.match(source, seed, cancelled=lambda: True)['reason'] == 'SUPERSEDED'
    outcome = reference.match(source, seed, validation_xyz=validation)
    assert outcome['success'], outcome
    assert np.linalg.norm(np.array(outcome['pose'][:3]) - truth[:3]) < 0.12, outcome
    assert abs(angle_delta(outcome['pose'][3], truth[3])) < math.radians(2.0), outcome
    assert reference.match(source, [30, 0, 0, 0])['reason'] == 'MAP_OUT_OF_COVERAGE'
    assert reference.match(np.zeros((15, 3)), truth)['reason'] == 'INSUFFICIENT_POINTS'

    floor = observation(surface(2, 0.0, bounds), truth, random)
    rejected = reference.match(floor, truth, mode='single')
    assert rejected['reason'] == 'DEGENERATE_GEOMETRY', rejected
    rejected = reference.match(source, truth, mode='single', validation_xyz=floor)
    assert rejected['reason'] == 'VALIDATION_FAILED', rejected

    blocked = LocalizationMap(np.vstack([walls, surface(0, 0.58, bounds)]))
    rejected = blocked.match(source, truth, mode='single')
    assert rejected['reason'] == 'FREE_SPACE_CONFLICT', rejected

    identical = room([(-1.3, 1.3), (-1.15, 1.7), (0.0, 2.3)])
    repeated = LocalizationMap(np.vstack([identical, identical + [3.6, 0.0, 0.0]]))
    source = observation(identical, truth, random)
    rejected = repeated.match(source, truth + [1.8, 0.0, 0.0, 0.0])
    assert rejected['reason'] == 'AMBIGUOUS', rejected
    stacked = LocalizationMap(np.vstack([identical, identical + [0.0, 0.0, 3.75]]))
    rejected = stacked.match(source, truth, validation_xyz=observation(identical, truth, random))
    assert rejected['reason'] == 'AMBIGUOUS_FLOOR', rejected
    print('Map registration recovery and rejection checks passed.')


if __name__ == '__main__':
    main()
