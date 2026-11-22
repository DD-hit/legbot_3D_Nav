"""Report registered LiDAR returns in the forward corridor; no motion interfaces."""
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    rclpy.init()
    node = rclpy.create_node('d1_front_observation')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    last_report = 0.

    previous = None

    def cloud(incoming):
        nonlocal last_report, previous
        msg, previous = previous, incoming
        if msg is None:
            return
        now = time.monotonic()
        if now-last_report < .5:
            return
        try:
            tf = buffer.lookup_transform('base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
        except TransformException as error:
            last_report = now
            print(f"等待点云对应的机身变换：{error}", flush=True)
            return
        last_report = now
        data = point_cloud2.read_points(msg,field_names=['x','y','z'])
        points = np.column_stack([data[k].ravel() for k in ('x','y','z')])
        q,p = tf.rotation,tf.translation
        axis = np.array([q.x, q.y, q.z])
        cross = 2 * np.cross(axis, points)
        body = points + q.w * cross + np.cross(axis, cross) + [p.x, p.y, p.z]
        mask = ((body[:,0]>.35)&(body[:,0]<2.)&(np.abs(body[:,1])<.30)&
                (body[:,2]>-.15)&(body[:,2]<.35))
        forward = body[mask,0]
        near = f'{forward.min():.2f} m' if forward.size else '无'
        print(f'前方走廊采样 {forward.size} 点，最近纵向距离 {near}',flush=True)
    node.create_subscription(PointCloud2,'/fast_lio/cloud_registered',cloud,qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
