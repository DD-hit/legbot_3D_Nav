from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    directory = Path(get_package_share_directory('d1_odom_bringup'))
    description = (directory / 'urdf/d1_edu.urdf').read_text()
    imu_joint = ET.fromstring(description).find("joint[@name='livox_mid360s_imu_joint']")
    origin = imu_joint.find('origin')
    if any(float(value) for value in origin.get('rpy').split()):
        raise ValueError('D1 LiDAR-to-IMU axes must match this model’s calibration')
    lidar_in_imu = [-float(value) for value in origin.get('xyz').split()]
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='fast_lio', executable='fastlio_mapping',
             parameters=[str(directory / 'config/mid360s.yaml'),
                         {'mapping.extrinsic_T': lidar_in_imu, 'common.body_frame': 'lidar_imu_link'}],
             remappings=[('/Odometry', '/fast_lio/odometry_sensor'), ('/cloud_registered', '/fast_lio/cloud_registered')]),
        Node(package='d1_odom_bringup', executable='fastlio_odom_adapter',
             parameters=[{'base_frame': 'base_link', 'sensor_frame': 'lidar_imu_link', 'lidar_frame': 'lidar_link'}],
             remappings=[('input', '/fast_lio/odometry_sensor'), ('output', '/fast_lio/odometry_base'),
                         ('lidar_output', '/fast_lio/odometry_lidar')]),
    ])
