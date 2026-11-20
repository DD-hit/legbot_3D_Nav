"""Run FAST-LIO as a selectable odometry provider for a platform profile."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration as Arg

from legbot_platform.navigation import include, read_profile
from legbot_platform.resources import fastlio_path, robot_path


def setup(context):
    robot = read_profile(context, 'robot_profile')
    config = robot['fastlio_config']
    return [include('fastlio.launch.py', {
        'use_sim_time': Arg('use_sim_time'), 'align_gravity': Arg('align_gravity'),
        'config': fastlio_path(Arg('robot_profile').perform(context), config), 'robot_profile': Arg('robot_profile'),
        **{key: robot[key] for key in ('sensor_frame', 'lidar_frame', 'base_frame', 'odom_frame')},
        'raw_cloud_topic': robot['raw_cloud'], 'imu_topic': robot['imu'], 'base_odom_topic': robot['odometry'],
        'registered_cloud_topic': robot['registered_cloud'], 'lidar_odom_topic': robot['lidar_odometry'],
    })]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_profile', default_value=robot_path('go2')),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('align_gravity', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
