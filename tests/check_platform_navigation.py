"""Run with an unused ROS_DOMAIN_ID and the Humble overlays sourced."""
import json
import os
import signal
import socket
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.request import urlopen

import numpy as np
import rclpy
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.utilities import evaluate_parameters
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool

from legbot_platform.calibration import sensor_joints


def check_calibration(robot, directory):
    robot.update(sensor_frame='test_imu', lidar_frame='test_lidar',
                 imu_in_lidar={'xyz': [0.07, -0.03, 0.04], 'rpy': [0.1, -0.2, 0.3]})
    robot['lidar_mount']['lidar_pitch'] = -0.42
    description = Path(get_package_share_directory('go2_description')) / 'xacro/robot.xacro'
    arguments = sensor_joints(robot, lambda _: '')
    model = ET.fromstring(xacro.process_file(str(description), mappings={**arguments, 'GAZEBO': 'true', 'CLASSIC': 'false'}).toxml())
    mount = model.find("joint[@name='livox_mount']")
    assert mount.find('child').get('link') == 'test_lidar'
    assert float(mount.find('origin').get('rpy').split()[1]) == -0.42
    joint = model.find("joint[@name='livox_imu_joint']")
    assert joint.find('parent').get('link') == 'test_lidar' and joint.find('child').get('link') == 'test_imu'
    translation = np.fromstring(joint.find('origin').get('xyz'), sep=' ')
    rotation = Rotation.from_euler('xyz', np.fromstring(joint.find('origin').get('rpy'), sep=' ')).as_matrix()
    profile_path = directory / 'sensor.yaml'
    profile_path.write_text(yaml.safe_dump(robot))
    launch_dir = Path(get_package_share_directory('legbot_bringup')) / 'launch'
    context = LaunchContext()
    context.launch_configurations['robot_profile'] = str(profile_path)
    sensor_launch = PythonLaunchDescriptionSource(str(launch_dir / 'sensor_odometry.launch.py')).get_launch_description(context)
    for declaration in sensor_launch.entities[:-1]:
        declaration.execute(context)
    from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
    sensor_include, = sensor_launch.entities[-1].execute(context)
    values = {key: perform_substitutions(context, normalize_to_list_of_substitutions(value))
              for key, value in sensor_include.launch_arguments}
    context.launch_configurations.update(values)
    odom_launch = sensor_include.launch_description_source.get_launch_description(context)
    for declaration in odom_launch.entities:
        if isinstance(declaration, DeclareLaunchArgument):
            declaration.execute(context)
    mapping, _ = odom_launch.entities[-1].execute(context)
    extrinsic = next(params for params in evaluate_parameters(context, mapping._Node__parameters)
                     if isinstance(params, dict) and 'mapping.extrinsic_R' in params)
    assert extrinsic['common.body_frame'] == robot['sensor_frame']
    assert np.allclose(rotation @ np.reshape(extrinsic['mapping.extrinsic_R'], (3, 3)), np.eye(3))
    assert np.allclose(rotation @ extrinsic['mapping.extrinsic_T'] + translation, 0)
    assert {element.text for element in model.findall('.//sensor/gz_frame_id')} >= {'test_lidar', 'test_imu'}


def check_graph(robot, directory):
    robot.update(ready_topic='/test_platform/status', cmd_vel_topic='/test_platform/velocity',
                 imu='/test_platform/imu', odometry='/test_platform/odom', registered_cloud='/test_platform/cloud',
                 lidar_odometry='/test_platform/lidar_pose', ready_timeout=0.8)
    path = directory / 'robot.yaml'
    path.write_text(yaml.safe_dump(robot))
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    rclpy.init()
    node = rclpy.create_node('platform_contract_observer')
    assert node.get_node_names() == [node.get_name()], 'use an isolated ROS domain'
    velocities = []
    node.create_subscription(Twist, robot['cmd_vel_topic'], lambda message: velocities.append(message), 10)
    platform = node.create_publisher(Bool, robot['ready_topic'], QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    with (directory / 'navigation.log').open('w+') as log:
        process = subprocess.Popen(['ros2', 'launch', 'legbot_bringup', 'navigation.launch.py',
                                    f'robot_profile:={path}', f'web_port:={port}'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            def wait_for(predicate, timeout=25):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline and process.poll() is None:
                    rclpy.spin_once(node, timeout_sec=.1)
                    try:
                        with urlopen(f'http://127.0.0.1:{port}/api/state', timeout=.3) as stream:
                            state = json.load(stream)
                            if 'standing' in state and predicate(state):
                                return
                    except OSError:
                        continue
                raise AssertionError('Platform graph or status timeout')

            wait_for(lambda state: not state['standing'] and velocities and platform.get_subscription_count() > 0
                     and all(node.count_subscribers(robot[key]) for key in ('imu','odometry','registered_cloud','lidar_odometry')))
            names = node.get_node_names()
            assert 'navigation_velocity_guard' in names and 'map_localizer' in names
            assert not {'go2_demo_sequencer', 'go2_command_adapter', 'controller_manager', 'laser_mapping'} & set(names)
            platform.publish(Bool(data=True))
            wait_for(lambda state: state['standing'] and not state['ready'])
            wait_for(lambda state: not state['standing'], timeout=3)
            assert all((msg.linear.x, msg.linear.y, msg.angular.z) == (0, 0, 0) for msg in velocities)
        except BaseException:
            log.seek(0)
            print(log.read()[-5000:])
            raise
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    robot = yaml.safe_load((Path(get_package_share_directory('legbot_bringup')) / 'config/robot.yaml').read_text())
    with tempfile.TemporaryDirectory() as directory:
        check_calibration(robot, Path(directory))
        check_graph(robot, Path(directory))
    print('PASS tilted mount, inverse IMU calibration, external navigation inputs, standby output and readiness timeout')
