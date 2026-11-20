import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    map_path = os.path.join(get_package_share_directory('pct_planner'), 'src', 'pcd', 'building2_9.pcd')
    arguments = [
        ('map_path', map_path, str), ('use_sim_time', 'false', bool),
        ('map_frame', 'map', str), ('odom_frame', 'odom', str), ('base_frame', 'base', str),
        ('lidar_frame', 'livox_link', str),
        ('cloud_topic', '/fast_lio/cloud_registered', str),
        ('imu_topic', '', str),
        ('initial_z_mode', 'floor', str), ('base_height', '0.32', float), ('floor_height', '0.0', float),
        ('mode', 'multi', str), ('xy_radius', '2.5', float), ('yaw_radius', '45.0', float),
        ('scan_frames', '4', int), ('validation_frames', '2', int),
        ('max_scan_translation', '0.15', float), ('max_scan_rotation_deg', '8.0', float),
        ('request_timeout', '25.0', float), ('sensor_timeout', '3.0', float),
        ('tracking_xy_tolerance', '0.16', float), ('tracking_z_tolerance', '0.17', float),
        ('tracking_yaw_deg', '5.0', float), ('tracking_grace', '1.2', float), ('tracking_max_speed', '1.5', float),
    ]
    declarations = [DeclareLaunchArgument(name, default_value=default) for name, default, _ in arguments]
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=kind) for name, _, kind in arguments}
    return LaunchDescription(declarations + [
        Node(package='legbot_bringup', executable='map_localizer', output='screen', parameters=[parameters])
    ])
