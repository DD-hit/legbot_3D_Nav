import os

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from legbot_platform.map_profiles import load_map


def include(filename, arguments, condition=None):
    path = os.path.join(share('legbot_bringup'), 'launch', filename)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=arguments.items(), condition=condition)


def read_profile(context, name):
    path = Arg(name).perform(context)
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def navigation_arguments(web_default, sim_time_default, map_default=None):
    config_dir = share('legbot_config')
    return [
        DeclareLaunchArgument('robot_profile', default_value=os.path.join(config_dir, 'robots', 'go2.yaml')),
        DeclareLaunchArgument('map_profile', default_value=map_default or os.path.join(config_dir, 'maps', 'building2_9', 'map.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value=sim_time_default, choices=['true', 'false']),
        DeclareLaunchArgument('web_console', default_value=web_default, choices=['true', 'false']),
        DeclareLaunchArgument('web_port', default_value='8765'),
        DeclareLaunchArgument('initial_z_mode', default_value='floor', choices=['floor', 'base', 'rviz_floor']),
        DeclareLaunchArgument('floor_height', default_value='0.0'),
        DeclareLaunchArgument('goal_x', default_value='2.0'),
        DeclareLaunchArgument('goal_y', default_value='-3.0'),
        DeclareLaunchArgument('goal_z', default_value='4.5'),
        DeclareLaunchArgument('finish_dist', default_value='0.07'),
        DeclareLaunchArgument('goal_settle_time', default_value='0.60'),
        DeclareLaunchArgument('goal_min_walk_speed', default_value='', description='Override platform final approach speed floor (m/s)'),
    ]


def navigation_actions(context, robot, *, localized=True, route_frame=None, mission_options=None, demo_map=False):
    web = Arg('web_console').perform(context) == 'true'
    building = load_map(Arg('map_profile').perform(context), web=web, demo=demo_map)
    frame = route_frame or building['frame']
    if web and (not localized or Arg('initial_z_mode').perform(context) != 'floor'):
        raise ValueError('Web navigation requires map localization and floor-relative initial poses')
    clock = Arg('use_sim_time')
    ready = robot.get('ready_topic', '/platform/ready')
    velocity = robot.get('cmd_vel_topic', '/cmd_vel')
    requested = '/navigation/requested_velocity'
    actions = []
    if localized:
        if velocity == requested:
            raise ValueError('cmd_vel_topic must differ from the internal velocity request topic')
        actions.extend([
            include('map_localizer.launch.py', {
                'use_sim_time': clock, 'initial_z_mode': Arg('initial_z_mode'), 'floor_height': Arg('floor_height'),
                'map_path': building['pcd'], 'map_frame': frame,
                'odom_frame': robot['odom_frame'], 'base_frame': robot['base_frame'],
                'lidar_frame': robot['lidar_frame'], 'base_height': str(robot['base_height']),
                'imu_topic': robot['imu'], 'cloud_topic': robot['registered_cloud'],
                **({'xy_radius': '1.0', 'yaw_radius': '30.0'} if web else {}),
            }),
            Node(package='legbot_bringup', executable='navigation_velocity_guard', parameters=[{
                'use_sim_time': ParameterValue(clock, value_type=bool), 'require_map_localization': True,
                'odom_topic': robot['odometry'], 'cloud_topic': robot['registered_cloud'],
                'navigation_timeout': float(robot.get('navigation_timeout', 0.5)),
                'command_timeout': float(robot.get('command_timeout', 0.5)),
                'max_vx': float(robot['max_speed']), 'max_vy': float(robot['max_lateral_speed']),
                'max_yaw_rate': float(robot['max_yaw_rate']),
            }], remappings=[('requested', requested), ('output', velocity)]),
        ])
    actions.extend([
        include('pct_plan.launch.py', {
            'use_sim_time': clock, 'tomogram': building.get('demo_tomogram', building['tomogram']) if demo_map else building['tomogram'],
            'path_topic': '/pct_path_raw', 'frame_id': frame, 'use_localized_start': str(localized).lower(),
            'runtime_goals': str(web).lower(), 'goal_x': Arg('goal_x'), 'goal_y': Arg('goal_y'), 'goal_z': Arg('goal_z'),
        }),
        include('scan.launch.py', {
            'use_sim_time': clock, 'navi_mode': '3', 'global_path_topic': '/pct_path',
            'cancel_on_empty_path': str(localized).lower(), 'require_route_stamp': str(localized).lower(),
            'frame_id': robot['odom_frame'], 'manual_goal_use_message_z': 'true',
            'finish_dist': Arg('finish_dist'), 'goal_settle_time': Arg('goal_settle_time'),
            'respect_goal_heading': str(web).lower(), 'min_walk_speed': str(robot.get('min_walk_speed', 0.0)),
            'goal_min_walk_speed': Arg('goal_min_walk_speed').perform(context) or str(robot.get('goal_min_walk_speed', 0.0)),
            'odom_topic': robot['odometry'],
            'cloud_topic': robot['registered_cloud'], 'sensor_pose_topic': robot['lidar_odometry'],
            'cmd_vel_topic': requested if localized else velocity,
            'robot_radius': str(robot['footprint_radius']), 'robot_half_length': str(robot['footprint_offset']),
            'max_vel': str(robot['max_speed']), 'max_vy': str(robot['max_lateral_speed']), 'max_vyaw': str(robot['max_yaw_rate']),
            **{key: str(robot[key]).lower() for key in ('max_acc', 'min_stair_speed', 'lidar_horizontal_fov_deg',
                                                        'lidar_clear_outside_fov') if key in robot},
        }),
        Node(package='legbot_bringup', executable='pct_path_mission', name='pct_path_mission', output='screen', parameters=[{
            'use_sim_time': ParameterValue(clock, value_type=bool), 'input_topic': '/pct_path_raw', 'output_topic': '/pct_path',
            'goal_topic': '/scan/goal', 'odom_topic': robot['odometry'], 'ready_topic': ready,
            'ready_timeout': float(robot.get('ready_timeout', 2.0)) if localized else 0.0,
            'frame_id': robot['odom_frame'], 'base_frame': robot['base_frame'], 'pct_frame_id': frame,
            'use_map_localization': localized, 'web_console': web, 'map_profile': Arg('map_profile'),
            'web_port': ParameterValue(Arg('web_port'), value_type=int), 'publish_map_tf': False,
            **(mission_options or {}),
        }]),
    ])
    return actions
