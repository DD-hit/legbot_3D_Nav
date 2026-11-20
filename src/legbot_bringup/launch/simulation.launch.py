"""GO2/HIMLoco on Gazebo Harmonic. Launching this file starts simulation."""
import os
import tempfile
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    cfg = lambda key: LaunchConfiguration(key).perform(context)
    use_ground_truth = cfg('navigation_source') == 'ground_truth'
    publish_ground_truth = cfg('publish_ground_truth') == 'true'
    diagnose_fastlio = cfg('diagnose_fastlio') == 'true'
    if use_ground_truth and not publish_ground_truth:
        raise RuntimeError(
            'navigation_source:=ground_truth requires publish_ground_truth:=true')
    if diagnose_fastlio and not publish_ground_truth:
        raise RuntimeError(
            'diagnose_fastlio:=true requires publish_ground_truth:=true')
    if diagnose_fastlio and use_ground_truth:
        raise RuntimeError(
            'diagnose_fastlio:=true is an evaluation mode for navigation_source:=fastlio')
    if cfg('publish_ground_truth') == 'false' and cfg('synthetic_stair_cloud') == 'true':
        raise RuntimeError('synthetic_stair_cloud requires publish_ground_truth:=true; use the simulated LiDAR with FAST-LIO otherwise')
    controller_files = {
        'himloco': 'gazebo.yaml',
        'legged_gym': 'legged_gym_controller.yaml',
        'stairs_trot': 'stairs_trot_controller.yaml',
        'robot_lab': 'robot_lab_controller.yaml',
        'go2_cts': 'go2_cts_controller.yaml',
        'mjlab_flat': 'mjlab_flat_controller.yaml',
        'moe_cts_77k': 'moe_cts_77k_controller.yaml',
    }
    controller_file = controller_files[cfg('policy_profile')]
    description = xacro.process_file(os.path.join(share('go2_description'), 'xacro/robot.xacro'),
                                     mappings={'GAZEBO': 'true', 'CLASSIC': 'false',
                                               'ENABLE_LIDAR': cfg('enable_lidar'),
                                               **{key: cfg(key) for key in ('lidar_x','lidar_y','lidar_z','lidar_roll','lidar_pitch','lidar_yaw')},
                                               'controller_config': os.path.join(share('go2_description'), 'config', controller_file)}).toxml()
    world = cfg('world')
    description_dir = tempfile.TemporaryDirectory(prefix='legbot-sim-')
    description_path = Path(description_dir.name) / 'robot.urdf'
    description_path.write_text(description, encoding='utf-8')
    gz_args = ['-r', '-v', '3']
    if cfg('gui') == 'false':
        gz_args.append('-s')
        if cfg('headless_rendering') == 'true':
            gz_args.append('--headless-rendering')
    gz_args.append(world)
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'tower', '-topic', '/go2/robot_description', '-name', 'go2',
                            '-x', cfg('x'), '-y', cfg('y'), '-z', cfg('z'), '-Y', cfg('yaw')])
    spawners = [Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=[name, '--controller-manager', '/controller_manager', '--controller-manager-timeout', '120'])
                for name in ['joint_state_broadcaster', 'imu_sensor_broadcaster', 'rl_quadruped_controller']]
    def next_on_success(next_actions):
        def callback(event, context):
            return next_actions if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='GO2 startup process failed'))]
        return callback
    bridge_arguments = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/livox/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/livox/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
    ]
    bridge_remappings = [('/livox/scan/points', cfg('raw_cloud_topic')), ('/livox/imu', cfg('imu_topic'))]
    if publish_ground_truth:
        bridge_arguments.append('/go2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry')
        bridge_remappings.append(('/go2/odometry', '/Odometry_gazebo'))
    # Do not publish a second odom -> base authority while FAST-LIO owns that
    # transform.  Gazebo pose TF is only part of the explicit truth pipeline.
    if use_ground_truth:
        bridge_arguments.append('/go2/pose_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')
        bridge_remappings.append(('/go2/pose_tf', '/tf'))
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'use_sim_time': True}], arguments=bridge_arguments,
        remappings=bridge_remappings)
    resource_path = os.path.join(share('legbot_bringup'), 'models')
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resource_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']
    package_share = share('legbot_bringup')
    workspace_root = os.path.abspath(os.path.join(package_share, '..', '..', '..', '..'))
    diagnostic_overlay = os.path.join(
        workspace_root, '.deps', 'diagnostic_updater_4.2.7', 'opt', 'ros', 'jazzy', 'lib')
    library_path = os.environ.get('LD_LIBRARY_PATH', '')
    bundled_library_dirs = [
        os.path.join(workspace_root, 'third_party', 'libtorch', 'lib'),
        os.path.join(workspace_root, 'third_party', 'onnxruntime', 'lib'),
        diagnostic_overlay,
    ]
    for directory in reversed(bundled_library_dirs):
        if os.path.isdir(directory):
            library_path = directory + (os.pathsep + library_path if library_path else '')
    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('LD_LIBRARY_PATH', library_path),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=lambda _: description_dir.cleanup())])),
        ExecuteProcess(cmd=['gz', 'sim', *gz_args], output='screen',
                       additional_env={'LEGBOT_SIM_DESCRIPTION_FILE': str(description_path)},
                       on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}],
             remappings=[('robot_description', '/go2/robot_description')]),
        bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=next_on_success([spawners[0]]))),
        RegisterEventHandler(OnProcessExit(target_action=spawners[0], on_exit=next_on_success([spawners[1]]))),
        RegisterEventHandler(OnProcessExit(target_action=spawners[1], on_exit=next_on_success([spawners[2]]))),
        spawn,
        *([Node(package='legbot_bringup', executable='cloud_transform',
                parameters=[{'use_sim_time': True, 'target_frame': 'odom'}])]
          if use_ground_truth else []),
        Node(package='legbot_bringup', executable='go2_command_adapter', parameters=[{
            'use_sim_time': True,
            'odom_topic': '/Odometry_gazebo' if use_ground_truth else cfg('base_odom_topic'),
            'cloud_topic': '/livox/points_world' if use_ground_truth else cfg('registered_cloud_topic'),
            # Gazebo is intentionally slower than wall time while FAST-LIO and
            # RViz are active. Avoid chopping a valid locomotion command between
            # simulated sensor frames.
            'command_timeout': 2.0,
            'navigation_timeout': 2.0,
            'require_navigation_data': cfg('require_navigation_data') == 'true',
            'require_map_localization': cfg('navigation_source') == 'map_localized',
        }]),
        *([Node(package='legbot_bringup', executable='stair_cloud_sim', output='screen',
               parameters=[{'use_sim_time': True, 'odom_topic': '/Odometry_gazebo',
                            'scene_type': cfg('synthetic_scene'),
                            'include_risers': cfg('synthetic_include_risers') == 'true'}])]
          if cfg('synthetic_stair_cloud') == 'true' else []),
        *([Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'])] if use_ground_truth else []),
        *([Node(package='legbot_bringup', executable='fastlio_truth_monitor', output='screen',
               parameters=[{'use_sim_time': True}])]
          if diagnose_fastlio else []),
    ]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in {
            'lidar_x':'0.15','lidar_y':'0.0','lidar_z':'0.15','lidar_roll':'0.0','lidar_pitch':'0.0','lidar_yaw':'0.0',
            'raw_cloud_topic':'/livox/points_raw','imu_topic':'/livox/imu',
            'base_odom_topic':'/fast_lio/odometry_base','registered_cloud_topic':'/fast_lio/cloud_registered'}.items()],
        # FAST-LIO is the normal navigation source. Gazebo truth can be
        # published independently for evaluation without entering control.
        DeclareLaunchArgument('navigation_source', default_value='fastlio',
                              choices=['fastlio', 'ground_truth', 'map_localized']),
        DeclareLaunchArgument('publish_ground_truth', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('diagnose_fastlio', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('headless_rendering', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_lidar', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('require_navigation_data', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('synthetic_stair_cloud', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('synthetic_scene', default_value='stairs', choices=['stairs', 'stairs_obstacles', 'flat']),
        DeclareLaunchArgument('synthetic_include_risers', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('policy_profile', default_value='moe_cts_77k',
                              choices=['himloco', 'legged_gym', 'stairs_trot', 'robot_lab', 'go2_cts',
                                       'mjlab_flat', 'moe_cts_77k']),
        DeclareLaunchArgument('world', default_value=os.path.join(share('legbot_bringup'), 'worlds/Building.sdf')),
        # Building.sdf starts in the flat western half. Driving east first
        # exercises obstacle avoidance, then reaches the multi-storey stairs.
        DeclareLaunchArgument('x', default_value='-27.0'), DeclareLaunchArgument('y', default_value='6.0'),
        DeclareLaunchArgument('z', default_value='0.50'), DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
