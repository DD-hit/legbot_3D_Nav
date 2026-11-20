import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    config = LaunchConfiguration('config').perform(context)
    sensor_frame = LaunchConfiguration('sensor_frame').perform(context)
    align_gravity = LaunchConfiguration('align_gravity').perform(context) == 'true'
    value = lambda name: LaunchConfiguration(name).perform(context)
    settings = {'use_sim_time': use_sim_time, 'mapping.align_gravity': align_gravity,
                'sensor_queue_depth': int(value('sensor_queue_depth')), 'common.body_frame': sensor_frame}
    for parameter, argument in [('common.lid_topic','raw_cloud_topic'),('common.imu_topic','imu_topic'),('common.world_frame','odom_frame')]:
        if value(argument):
            settings[parameter] = value(argument)
    return [
        Node(package='fast_lio', executable='fastlio_mapping',
             parameters=[config, settings],
             remappings=[('/Odometry','/fast_lio/odometry_sensor'),
                         ('/cloud_registered',value('registered_cloud_topic')),
                         ('/cloud_registered_body','/fast_lio/cloud_registered_body'),
                         ('/Laser_map','/fast_lio/map'),
                         ('/path','/fast_lio/path')]),
        Node(package='legbot_bringup', executable='fastlio_odom_adapter',
             parameters=[{'use_sim_time': use_sim_time, 'sensor_frame': sensor_frame,
                          'lidar_frame': LaunchConfiguration('lidar_frame').perform(context),
                          'base_frame': value('base_frame')}],
             remappings=[('input','/fast_lio/odometry_sensor'),
                         ('output',value('base_odom_topic')),
                         ('lidar_output',value('lidar_odom_topic'))]),
    ]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key,value in {
            'raw_cloud_topic':'','imu_topic':'','odom_frame':'','base_frame':'base','sensor_queue_depth':'5',
            'registered_cloud_topic':'/fast_lio/cloud_registered','base_odom_topic':'/fast_lio/odometry_base',
            'lidar_odom_topic':'/fast_lio/odometry_lidar'}.items()],
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config', default_value=os.path.join(share('legbot_bringup'),'config/fastlio_real.yaml')),
        DeclareLaunchArgument('sensor_frame', default_value='livox_imu_link'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_link'),
        DeclareLaunchArgument('align_gravity', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
