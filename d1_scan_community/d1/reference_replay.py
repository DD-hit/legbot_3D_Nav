"""Publish the same two-point reference to the stock controller on a replay-only domain."""
import math
import os
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation

if os.environ.get('ROS_DOMAIN_ID') not in ('196','197','198') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Reserved for offline replay')
rclpy.init()
node = rclpy.create_node('community_reference_replay')
latest = []
def odom(message):
    latest[:] = [message.pose.pose]
node.create_subscription(Odometry,'/fast_lio/odometry_base',odom,qos_profile_sensor_data)
pub = node.create_publisher(Path,'/d1_community/initial_path',1)
start = time.monotonic()
while not latest or not pub.get_subscription_count() or time.monotonic()-start < 2:
    if time.monotonic()-start > 12:
        raise RuntimeError('no odometry or planner')
    rclpy.spin_once(node,timeout_sec=.05)
pose = latest[0]
q = pose.orientation
yaw = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]
x,y = pose.position.x,pose.position.y
goal = (x+2.3*math.cos(yaw), y+2.3*math.sin(yaw))
path = Path()
path.header.frame_id='odom'
path.header.stamp=node.get_clock().now().to_msg()
for a,b in [goal]:
    point=PoseStamped()
    point.header=path.header
    point.pose.position.x=a
    point.pose.position.y=b
    point.pose.position.z=pose.position.z-.4
    point.pose.orientation.w=1.
    path.poses.append(point)
pub.publish(path)
inside_since=None
while time.monotonic()-start < 55:
    rclpy.spin_once(node,timeout_sec=.04)
    pos=latest[0].position
    distance=math.hypot(pos.x-goal[0],pos.y-goal[1])
    if distance <= .15:
        inside_since=inside_since or time.monotonic()
        if time.monotonic()-inside_since >= .45:
            print(f'XY_REACHED error={distance:.3f}'); break
    else:
        inside_since=None
else:
    print(f'TIMEOUT error={distance:.3f}')
node.destroy_node()
rclpy.shutdown()
