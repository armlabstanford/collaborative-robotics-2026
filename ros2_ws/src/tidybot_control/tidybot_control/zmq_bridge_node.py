#!/usr/bin/env python3
"""
ZMQ Bridge Node — streams RGB + depth + camera pose from ROS2 to an external
GPU server for dense 3D mapping/localization.

Subscribes to:
    /camera/color/image_raw              (sensor_msgs/Image, RGB)
    /camera/aligned_depth_to_color/image_raw  (sensor_msgs/Image, 16UC1 mm)
    /camera/color/camera_info            (sensor_msgs/CameraInfo)
    /odom                                (nav_msgs/Odometry)
    TF: camera_color_optical_frame → odom

Publishes via ZMQ PUB on tcp://*:5555 using msgpack serialisation.

Usage:
    ros2 run tidybot_control zmq_bridge_node
    ros2 run tidybot_control zmq_bridge_node --ros-args -p port:=5555 -p jpeg_quality:=80
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry

import tf2_ros

import zmq
import msgpack
import cv2
import lz4.frame


def quat_to_matrix(q):
    """Quaternion (x, y, z, w) → 3×3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def transform_to_4x4(transform):
    """geometry_msgs/Transform → 4×4 numpy matrix."""
    t = transform.translation
    q = transform.rotation
    mat = np.eye(4)
    mat[:3, :3] = quat_to_matrix([q.x, q.y, q.z, q.w])
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


class ZmqBridgeNode(Node):
    def __init__(self):
        super().__init__('zmq_bridge_node')

        # Parameters
        self.declare_parameter('port', 5555)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('use_lz4', True)

        port = self.get_parameter('port').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        depth_topic = self.get_parameter('depth_topic').value
        self.use_lz4 = self.get_parameter('use_lz4').value

        # ZMQ publisher
        self.ctx = zmq.Context()
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.setsockopt(zmq.SNDHWM, 2)  # drop old frames if consumer is slow
        self.pub.bind(f'tcp://*:{port}')
        self.get_logger().info(f'ZMQ PUB bound on tcp://*:{port}')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.intrinsics = None  # [fx, fy, cx, cy]
        self.latest_rgb = None  # (stamp, np.array H×W×3 uint8)
        self.frame_count = 0

        # QoS – sensor data (best effort for camera streams)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriptions
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 10)
        self.create_subscription(
            Image, '/camera/color/image_raw', self._rgb_cb, sensor_qos)
        self.create_subscription(
            Image, depth_topic, self._depth_cb, sensor_qos)

        self.get_logger().info(
            f'ZMQ bridge started (depth: {depth_topic}, '
            f'TF: {self.camera_frame} → {self.target_frame})')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _info_cb(self, msg: CameraInfo):
        if self.intrinsics is not None:
            return
        self.intrinsics = [msg.k[0], msg.k[4], msg.k[2], msg.k[5]]
        self.get_logger().info(
            f'Intrinsics: fx={self.intrinsics[0]:.1f} fy={self.intrinsics[1]:.1f} '
            f'cx={self.intrinsics[2]:.1f} cy={self.intrinsics[3]:.1f}')

    def _rgb_cb(self, msg: Image):
        h, w = msg.height, msg.width
        if msg.encoding in ('rgb8', 'bgr8'):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            self.latest_rgb = (msg.header.stamp, arr)

    def _depth_cb(self, msg: Image):
        """Triggered on each depth frame – build and send the ZMQ message."""
        if self.intrinsics is None or self.latest_rgb is None:
            return

        rgb_stamp, rgb_bgr = self.latest_rgb
        h, w = msg.height, msg.width

        # Decode depth to uint16 mm
        if msg.encoding == '16UC1':
            depth_u16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        elif msg.encoding == '32FC1':
            depth_f32 = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            depth_u16 = (depth_f32 * 1000.0).clip(0, 65535).astype(np.uint16)
        else:
            self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}', once=True)
            return

        # Look up camera→odom transform
        stamp = msg.header.stamp
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        camera_to_world = transform_to_4x4(tf_stamped.transform)

        # Compress RGB as JPEG
        ok, rgb_jpeg = cv2.imencode(
            '.jpg', rgb_bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return

        # Compress depth with lz4 (or send raw)
        depth_bytes = depth_u16.tobytes()
        if self.use_lz4:
            depth_bytes = lz4.frame.compress(depth_bytes)

        # Build message
        frame = {
            'timestamp': stamp.sec + stamp.nanosec * 1e-9,
            'rgb_jpeg': rgb_jpeg.tobytes(),
            'depth': depth_bytes,
            'depth_shape': [h, w],
            'depth_lz4': self.use_lz4,
            'intrinsics': self.intrinsics,
            'camera_to_world': camera_to_world.tolist(),
        }

        packed = msgpack.packb(frame, use_bin_type=True)
        self.pub.send(packed, zmq.NOBLOCK)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Sent {self.frame_count} frames')

    # ------------------------------------------------------------------

    def destroy_node(self):
        self.pub.close()
        self.ctx.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZmqBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
