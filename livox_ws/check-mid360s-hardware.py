import math
import statistics
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def stamp(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


rclpy.init()
node = rclpy.create_node('mid360s_hardware_check')
clouds, imus = [], []
cloud_subscription = node.create_subscription(CustomMsg, '/livox/lidar', clouds.append, qos_profile_sensor_data)
imu_subscription = node.create_subscription(Imu, '/livox/imu', imus.append, qos_profile_sensor_data)
try:
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    for topic, messages in (('/livox/lidar', clouds), ('/livox/imu', imus)):
        stamps = [stamp(message) for message in messages]
        assert len(stamps) > 10, (topic, len(stamps))
        assert stamps[0] > 0 and all(a < b for a, b in zip(stamps, stamps[1:])), topic
        rate = (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0])
        print(topic, len(stamps), 'messages', round(rate, 2), 'Hz', {m.header.frame_id for m in messages}, flush=True)
    assert all(cloud.point_num > 0 and cloud.point_num == len(cloud.points) for cloud in clouds)
    print('points/frame median', statistics.median(cloud.point_num for cloud in clouds))
    print('static acceleration magnitude median', statistics.median(math.hypot(
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z) for message in imus))
    print('last cloud-IMU stamp difference ms', abs(stamp(clouds[-1]) - stamp(imus[-1])) / 1e6)
finally:
    node.destroy_node()
    rclpy.shutdown()
