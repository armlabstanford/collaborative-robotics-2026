#!/usr/bin/env python3
"""
Generates a colored PointCloud2 from aligned depth + RGB using camera intrinsics.
Publishes to /camera/points in the camera_color_optical_frame.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField


class PointCloudNode(Node):
    def __init__(self):
        super().__init__('pointcloud_node')

        self.declare_parameter('max_depth_m', 5.0)
        self.declare_parameter('decimation', 4)

        self.max_depth = self.get_parameter('max_depth_m').value
        self.decimation = self.get_parameter('decimation').value

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.frame_id = 'camera_color_optical_frame'

        self.latest_rgb = None

        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._info_cb, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self._rgb_cb, 10)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self._depth_cb, 10)

        self.pc_pub = self.create_publisher(PointCloud2, '/camera/points', 10)
        self.get_logger().info('PointCloud node started')

    def _info_cb(self, msg):
        if self.fx is not None:
            return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.frame_id = msg.header.frame_id
        self.get_logger().info(f'Got intrinsics: fx={self.fx:.1f} fy={self.fy:.1f}')

    def _rgb_cb(self, msg):
        self.latest_rgb = msg

    def _depth_cb(self, depth_msg):
        if self.fx is None or self.latest_rgb is None:
            return

        rgb_msg = self.latest_rgb
        h, w = depth_msg.height, depth_msg.width

        if depth_msg.encoding == '16UC1':
            depth_m = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(h, w).astype(np.float32) * 0.001
        elif depth_msg.encoding == '32FC1':
            depth_m = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(h, w)
        else:
            return

        if rgb_msg.encoding in ('rgb8', 'bgr8'):
            rgb = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(h, w, 3)
            if rgb_msg.encoding == 'bgr8':
                rgb = rgb[:, :, ::-1]
        else:
            return

        step = self.decimation
        depth_m = depth_m[::step, ::step]
        rgb = rgb[::step, ::step]

        u = np.arange(0, w, step, dtype=np.float32)
        v = np.arange(0, h, step, dtype=np.float32)
        u, v = np.meshgrid(u, v)

        valid = (depth_m > 0.01) & (depth_m < self.max_depth)
        z = depth_m[valid]
        x = (u[valid] - self.cx) * z / self.fx
        y = (v[valid] - self.cy) * z / self.fy

        r = rgb[:, :, 0][valid].astype(np.uint32)
        g = rgb[:, :, 1][valid].astype(np.uint32)
        b = rgb[:, :, 2][valid].astype(np.uint32)
        rgb_float = ((r << 16) | (g << 8) | b).view(np.float32)

        n = len(z)
        pts = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
        pts['x'] = x
        pts['y'] = y
        pts['z'] = z
        pts['rgb'] = rgb_float

        msg = PointCloud2()
        msg.header.stamp = depth_msg.header.stamp
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.data = pts.tobytes()
        msg.is_dense = True
        self.pc_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PointCloudNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
