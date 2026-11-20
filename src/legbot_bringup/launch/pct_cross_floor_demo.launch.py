"""One-command Gazebo + PCT + SCAN cross-floor mission."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, launch_file, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share(package), 'launch', launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition)


def setup(context):
    def profile(argument):
        with open(LaunchConfiguration(argument).perform(context), encoding='utf-8') as stream:
            return yaml.safe_load(stream)
    robot = profile('robot_profile')
    building = profile('map_profile')
    profile_dir = os.path.dirname(os.path.abspath(LaunchConfiguration('map_profile').perform(context)))
    for key in ('tomogram', 'demo_tomogram', 'pcd'):
        if building.get(key, '').startswith('./'):
            building[key] = os.path.join(profile_dir, building[key])
    navigation_source = LaunchConfiguration('navigation_source').perform(context)
    use_ground_truth = navigation_source == 'ground_truth'
    map_localized = navigation_source == 'map_localized'
    web_console = LaunchConfiguration('web_console').perform(context) == 'true'
    if web_console and (not map_localized or LaunchConfiguration('initial_z_mode').perform(context) != 'floor'):
        raise ValueError('web_console requires navigation_source:=map_localized and initial_z_mode:=floor')
    pct_frame = building['frame'] if map_localized else 'building_pct'
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    legacy_geometry = {} if map_localized else {
        'building_x': 13.0, 'building_y': 0.0, 'building_z': 0.0, 'building_yaw': 0.0,
        'spawn_x': ParameterValue(spawn_x, value_type=float),
        'spawn_y': ParameterValue(spawn_y, value_type=float),
        'spawn_z': ParameterValue(spawn_z, value_type=float),
        'spawn_yaw': ParameterValue(spawn_yaw, value_type=float),
    }

    if use_ground_truth:
        odom_topic = '/Odometry_gazebo'
        cloud_topic = '/livox/points_world'
        sensor_pose_topic = '/go2/lidar_pose'
        localization_label = 'Gazebo ground truth'
        rviz_config = os.path.join(
            share('legbot_bringup'), 'rviz', 'scan_stairs_demo.rviz')
    else:
        odom_topic = robot['odometry']
        cloud_topic = robot['registered_cloud']
        sensor_pose_topic = robot['lidar_odometry']
        localization_label = 'FAST-LIO'
        rviz_config = os.path.join(
            share('legbot_bringup'), 'rviz', 'map_localization.rviz' if map_localized else 'scan_fastlio_flat.rviz')

    actions = [
        include('legbot_bringup', 'simulation.launch.py', {
            'gui': gui,
            'x': spawn_x,
            'y': spawn_y,
            'z': spawn_z,
            'yaw': spawn_yaw,
            **{key: str(value) for key, value in robot['lidar_mount'].items()},
            'raw_cloud_topic': robot['raw_cloud'], 'imu_topic': robot['imu'],
            'base_odom_topic': robot['odometry'], 'registered_cloud_topic': robot['registered_cloud'],
            'navigation_source': navigation_source,
            'publish_ground_truth': 'true' if use_ground_truth else LaunchConfiguration('publish_ground_truth'),
        }),
    ]
    if not use_ground_truth:
        actions.append(include('legbot_bringup', 'fastlio.launch.py', {
            'use_sim_time': use_sim_time,
            'align_gravity': 'true' if map_localized else 'false',
            'config': os.path.join(
                share('legbot_bringup'), 'config', robot['fastlio_config']),
            'sensor_frame': robot['sensor_frame'], 'lidar_frame': robot['lidar_frame'],
            'base_frame': robot['base_frame'], 'odom_frame': robot['odom_frame'],
            'raw_cloud_topic': robot['raw_cloud'], 'imu_topic': robot['imu'],
            'base_odom_topic': odom_topic, 'registered_cloud_topic': cloud_topic, 'lidar_odom_topic': sensor_pose_topic,
        }))
    if map_localized:
        actions.append(include('legbot_bringup', 'map_localizer.launch.py', {
            'use_sim_time': use_sim_time,
            'initial_z_mode': LaunchConfiguration('initial_z_mode'),
            'floor_height': LaunchConfiguration('floor_height'),
            'map_path': os.path.join(share('pct_planner'), building['pcd']),
            'map_frame': pct_frame, 'odom_frame': robot['odom_frame'], 'base_frame': robot['base_frame'],
            'lidar_frame': robot['lidar_frame'], 'base_height': str(robot['base_height']),
            'imu_topic': robot['imu'],
            'cloud_topic': cloud_topic,
            **({'xy_radius': '1.0', 'yaw_radius': '30.0'} if web_console else {}),
        }))

    # The normal six-terminal workflow starts this coordinator only after all
    # ros2_control spawners finish. Preserve that ordering in the all-in-one
    # launch so its first mode pulse is never lost.
    actions.extend([
        TimerAction(period=8.0, actions=[include(
            'legbot_bringup', 'go2_demo_control.launch.py', {
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                'cloud_topic': cloud_topic,
                'localization_label': localization_label,
                'stand_on_plan_loss': 'true' if map_localized else 'false',
                'hold_stance_at_goal': 'true',
                'completion_topic': '/scan/leg_complete' if web_console else '/scan/mission_complete',
            })]),
        include('legbot_bringup', 'pct_plan.launch.py', {
            'use_sim_time': use_sim_time,
            'tomogram': building['tomogram'] if web_console else building.get('demo_tomogram', building['tomogram']),
            'path_topic': '/pct_path_raw',
            'frame_id': pct_frame,
            'use_localized_start': 'true' if map_localized else 'false',
            'runtime_goals': 'true' if web_console else 'false',
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'goal_z': LaunchConfiguration('goal_z'),
        }),
        include('legbot_bringup', 'scan.launch.py', {
            'use_sim_time': use_sim_time,
            'navi_mode': '3',
            'global_path_topic': '/pct_path',
            'cancel_on_empty_path': 'true' if map_localized else 'false',
            # pct_plan also declares a generic frame_id argument.  Pass this
            # explicitly so the include cannot leak building_pct into SCAN's
            # grid-map and marker headers.
            'frame_id': 'odom' if use_ground_truth else robot['odom_frame'],
            'manual_goal_use_message_z': 'true',
            'finish_dist': LaunchConfiguration('finish_dist'),
            'goal_settle_time': LaunchConfiguration('goal_settle_time'),
            'respect_goal_heading': 'true' if web_console else 'false',
            'min_walk_speed': '0.12',
            'goal_min_walk_speed': LaunchConfiguration('goal_min_walk_speed'),
            'odom_topic': odom_topic,
            'cloud_topic': cloud_topic,
            'sensor_pose_topic': sensor_pose_topic,
            'robot_radius': str(robot['footprint_radius']), 'robot_half_length': str(robot['footprint_offset']),
            'max_vel': str(robot['max_speed']), 'max_vy': str(robot['max_lateral_speed']),
            'max_vyaw': str(robot['max_yaw_rate']),
        }),
        Node(
            package='legbot_bringup',
            executable='pct_path_mission',
            name='pct_path_mission',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'input_topic': '/pct_path_raw',
                'output_topic': '/pct_path',
                'goal_topic': '/scan/goal',
                'odom_topic': odom_topic,
                'ready_topic': '/go2/demo_ready',
                'frame_id': 'odom' if use_ground_truth else robot['odom_frame'],
                'base_frame': robot['base_frame'],
                'pct_frame_id': pct_frame,
                'use_map_localization': map_localized,
                'web_console': web_console,
                'map_profile': LaunchConfiguration('map_profile'),
                'web_port': ParameterValue(LaunchConfiguration('web_port'), value_type=int),
                # Ground-truth odom is the Gazebo world frame, so its map TF
                # is published immediately below.  FAST-LIO still needs the
                # mission bridge's start-pose calibration.
                'publish_map_tf': navigation_source == 'fastlio',
                **legacy_geometry,
            }],
        ),
        *([Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='building_pct_ground_truth_tf',
            output='screen',
            arguments=[
                '--x', '13.0', '--y', '0.0', '--z', '0.0',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'odom', '--child-frame-id', 'building_pct',
            ],
        )] if use_ground_truth else []),
        include(
            'legbot_bringup', 'scan_rviz.launch.py',
            {'use_sim_time': use_sim_time, 'config': rviz_config},
            condition=IfCondition(rviz)),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_profile', default_value=os.path.join(share('legbot_bringup'), 'config', 'robot.yaml')),
        DeclareLaunchArgument('map_profile', default_value=os.path.join(share('legbot_bringup'), 'config', 'map.yaml')),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('web_console', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('web_port', default_value='8765'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'navigation_source', default_value='ground_truth',
            choices=['ground_truth', 'fastlio', 'map_localized'],
            description=(
                'Ground truth is the reliable default for the long simulated cross-floor run; '
                'map_localized registers FAST-LIO scans against the building PCD after /initialpose.')),
        DeclareLaunchArgument('publish_ground_truth', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('initial_z_mode', default_value='floor', choices=['floor', 'base', 'rviz_floor']),
        DeclareLaunchArgument('floor_height', default_value='0.0'),
        DeclareLaunchArgument('goal_x', default_value='2.0'),
        DeclareLaunchArgument('goal_y', default_value='-3.0'),
        DeclareLaunchArgument('goal_z', default_value='4.5'),
        DeclareLaunchArgument('finish_dist', default_value='0.07',
                              description='Horizontal goal tolerance in metres'),
        DeclareLaunchArgument('goal_settle_time', default_value='0.60',
                              description='Seconds of stable odometry inside the goal tolerance'),
        DeclareLaunchArgument('goal_min_walk_speed', default_value='0.20',
                              description='Minimum commanded speed on the final approach in metres per second'),
        DeclareLaunchArgument('spawn_x', default_value=PythonExpression([
            "'7.5' if '", LaunchConfiguration('navigation_source'), "' == 'map_localized' else '-27.0'"])),
        DeclareLaunchArgument('spawn_y', default_value='6.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.50'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
