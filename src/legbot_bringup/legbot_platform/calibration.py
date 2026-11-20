import math
from pathlib import Path

import yaml
from scipy.spatial.transform import Rotation


def imu_pose(profile):
    pose = profile.get('imu_in_lidar')
    if pose is None:
        return None
    for key in ('xyz', 'rpy'):
        if len(pose[key]) != 3 or not all(math.isfinite(value) for value in pose[key]):
            raise ValueError(f'imu_in_lidar.{key} must contain three finite numbers')
    return pose


def sensor_profile(path):
    if not path:
        return {}
    with Path(path).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def sensor_joints(profile, override):
    pose = imu_pose(profile)
    joints = {}
    if pose is not None:
        joints.update({'imu_in_lidar_' + key: ' '.join(str(value) for value in pose[key]) for key in ('xyz', 'rpy')})
        joints['lidar_imu_frame'] = profile['sensor_frame']
    if 'lidar_frame' in profile:
        joints['lidar_frame'] = profile['lidar_frame']
    for key in ('lidar_x', 'lidar_y', 'lidar_z', 'lidar_roll', 'lidar_pitch', 'lidar_yaw'):
        value = override(key) or profile.get('lidar_mount', {}).get(key)
        if value is not None and value != '':
            if not math.isfinite(float(value)):
                raise ValueError(f'{key} must be finite')
            joints[key] = str(value)
    return joints


def lidar_in_imu(pose):
    if pose is None:
        return {}
    inverse = Rotation.from_euler('xyz', pose['rpy']).inv()
    return {'mapping.extrinsic_R': inverse.as_matrix().ravel().tolist(),
            'mapping.extrinsic_T': (-inverse.apply(pose['xyz'])).tolist()}
