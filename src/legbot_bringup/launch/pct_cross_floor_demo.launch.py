"""GO2 simulation and sensor adapters for the shared navigation stack."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as Arg, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution

from legbot_platform.map_profiles import SIMULATION_MAP
from legbot_platform.navigation import include, navigation_actions, navigation_arguments, read_profile


def setup(context):
    robot = read_profile(context, 'robot_profile')
    source = Arg('navigation_source').perform(context)
    truth = source == 'ground_truth'
    localized = source == 'map_localized'
    web = Arg('web_console').perform(context) == 'true'
    simulation = include('simulation.launch.py', {
        'world': Arg('world'), 'world_name': Arg('world_name'),
        'gui': Arg('gui'), **{axis: Arg('spawn_' + axis) for axis in ('x', 'y', 'z', 'yaw')},
        'robot_profile': Arg('robot_profile'), 'raw_cloud_topic': robot['raw_cloud'], 'imu_topic': robot['imu'],
        'base_odom_topic': robot['odometry'], 'registered_cloud_topic': robot['registered_cloud'],
        'cmd_vel_topic': robot.get('cmd_vel_topic', '/cmd_vel'), 'navigation_source': source,
        'publish_ground_truth': 'true' if truth else Arg('publish_ground_truth'),
    })
    actions = [simulation]
    if truth:
        robot.update(odometry='/Odometry_gazebo', registered_cloud='/livox/points_world',
                     lidar_odometry='/go2/lidar_pose', odom_frame='odom')
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name='building_pct_ground_truth_tf', arguments=[
                                '--x', '13.0', '--y', '0.0', '--z', '0.0', '--yaw', '0.0', '--pitch', '0.0',
                                '--roll', '0.0', '--frame-id', 'odom', '--child-frame-id', 'building_pct']))
    else:
        actions.append(include('sensor_odometry.launch.py', {
            'use_sim_time': Arg('use_sim_time'), 'robot_profile': Arg('robot_profile'),
            'align_gravity': str(localized).lower(),
        }))
    actions.append(TimerAction(period=8.0, actions=[include('go2_demo_control.launch.py', {
        'use_sim_time': Arg('use_sim_time'), 'odom_topic': robot['odometry'], 'cloud_topic': robot['registered_cloud'],
        'localization_label': 'Gazebo ground truth' if truth else 'FAST-LIO',
        'platform_ready_topic': robot.get('ready_topic', '/platform/ready'),
        'stand_on_plan_loss': str(localized).lower(), 'hold_stance_at_goal': 'true',
        'completion_topic': '/scan/leg_complete' if web else '/scan/mission_complete',
    })]))
    robot.setdefault('min_walk_speed', 0.12)
    robot.setdefault('goal_min_walk_speed', 0.20)
    legacy = {} if localized else {
        'building_x': 13.0, 'building_y': 0.0, 'building_z': 0.0, 'building_yaw': 0.0,
        'publish_map_tf': source == 'fastlio',
        **{'spawn_' + axis: ParameterValue(Arg('spawn_' + axis), value_type=float) for axis in ('x', 'y', 'z', 'yaw')},
    }
    actions.extend(navigation_actions(context, robot, localized=localized,
                                      route_frame=None if localized else 'building_pct', mission_options=legacy, demo_map=not web))
    view = 'scan_stairs_demo.rviz' if truth else ('map_localization.rviz' if localized else 'scan_fastlio_flat.rviz')
    actions.append(include('scan_rviz.launch.py', {'use_sim_time': Arg('use_sim_time'),
                           'config': os.path.join(share('legbot_bringup'), 'rviz', view)}, condition=IfCondition(Arg('rviz'))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('navigation_source', default_value='ground_truth', choices=['ground_truth', 'fastlio', 'map_localized']),
        *navigation_arguments('false', 'true', PathJoinSubstitution([
            share('legbot_config'), 'maps', PythonExpression([
                repr(SIMULATION_MAP), " if '", Arg('navigation_source'), "' == 'map_localized' else 'building2_9'"]), 'map.yaml'])),
        DeclareLaunchArgument('world', default_value=os.path.join(share('legbot_bringup'), 'worlds/Building.sdf')),
        DeclareLaunchArgument('world_name', default_value='tower'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('publish_ground_truth', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('spawn_x', default_value=PythonExpression([
            "'7.5' if '", Arg('navigation_source'), "' == 'map_localized' else '-27.0'"])),
        DeclareLaunchArgument('spawn_y', default_value='6.0'), DeclareLaunchArgument('spawn_z', default_value='0.50'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'), OpaqueFunction(function=setup),
    ])
