#!/usr/bin/env python3
"""
Generates colored PointCloud2 from aligned depth + RGB using camera intrinsics.

Supports a single camera (default) or dual cameras combined via TF.

Single camera:
    Publishes to /camera/points in the camera's optical frame.

Dual camera (use_top_camera:=true):
    Combines pan-tilt + top camera into a single cloud published to
    /camera/combined_points in the base_link frame.  Each camera's
    cloud is also still published individually on /camera/points and
    /top_camera/points.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField

# tf2 imports for dual-camera transform
try:
    from tf2_ros import Buffer, TransformListener
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False


PC2_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]


class CameraState:
    """Holds intrinsics and latest RGB for one camera."""
    def __init__(self):
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.frame_id = None
        self.latest_rgb = None


def backproject(depth_msg, rgb_msg, cam, max_depth, decimation):
    """Backproject depth + RGB into Nx4 structured array (x, y, z, rgb)."""
    h, w = depth_msg.height, depth_msg.width

    if depth_msg.encoding == '16UC1':
        depth_m = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(h, w).astype(np.float32) * 0.001
    elif depth_msg.encoding == '32FC1':
        depth_m = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(h, w)
    else:
        return None

    if rgb_msg.encoding in ('rgb8', 'bgr8'):
        rgb = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(h, w, 3)
        if rgb_msg.encoding == 'bgr8':
            rgb = rgb[:, :, ::-1]
    else:
        return None

    step = decimation
    depth_m = depth_m[::step, ::step]
    rgb = rgb[::step, ::step]

    u = np.arange(0, w, step, dtype=np.float32)
    v = np.arange(0, h, step, dtype=np.float32)
    u, v = np.meshgrid(u, v)

    valid = (depth_m > 0.01) & (depth_m < max_depth)
    z = depth_m[valid]
    x = (u[valid] - cam.cx) * z / cam.fx
    y = (v[valid] - cam.cy) * z / cam.fy

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
    return pts


def make_pc2_msg(pts, stamp, frame_id):
    """Build a PointCloud2 message from structured array."""
    n = len(pts)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n
    msg.fields = PC2_FIELDS
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.data = pts.tobytes()
    msg.is_dense = True
    return msg


def transform_points(pts, tf_stamped):
    """Apply a TF transform to structured point array in-place, return new array."""
    t = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation

    # Quaternion to rotation matrix
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz),  2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),       1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float32)
    tvec = np.array([t.x, t.y, t.z], dtype=np.float32)

    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1)  # (N, 3)
    xyz_tf = xyz @ R.T + tvec

    out = np.copy(pts)
    out['x'] = xyz_tf[:, 0]
    out['y'] = xyz_tf[:, 1]
    out['z'] = xyz_tf[:, 2]
    return out


class PointCloudNode(Node):
    def __init__(self):
        super().__init__('pointcloud_node')

        self.declare_parameter('max_depth_m', 5.0)
        self.declare_parameter('decimation', 4)
        self.declare_parameter('use_top_camera', False)
        self.declare_parameter('combined_frame', 'base_link')
        self.declare_parameter('camera_prefix', 'camera')

        self.max_depth = self.get_parameter('max_depth_m').value
        self.decimation = self.get_parameter('decimation').value
        self.use_top_camera = self.get_parameter('use_top_camera').value
        self.combined_frame = self.get_parameter('combined_frame').value
        cam_prefix = self.get_parameter('camera_prefix').value

        # Primary camera (subscribes to /<camera_prefix>/... topics)
        self.primary = CameraState()
        self.create_subscription(CameraInfo, f'/{cam_prefix}/color/camera_info', self._primary_info_cb, 10)
        self.create_subscription(Image, f'/{cam_prefix}/color/image_raw', self._primary_rgb_cb, 10)
        self.create_subscription(Image, f'/{cam_prefix}/aligned_depth_to_color/image_raw', self._primary_depth_cb, 10)
        self.pc_pub = self.create_publisher(PointCloud2, f'/{cam_prefix}/points', 10)

        # Top camera (optional)
        self.top = CameraState()
        if self.use_top_camera:
            if not HAS_TF2:
                self.get_logger().error('tf2_ros not available — cannot combine cameras')
                raise RuntimeError('tf2_ros required for dual camera mode')

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.create_subscription(CameraInfo, '/top_camera/color/camera_info', self._top_info_cb, 10)
            self.create_subscription(Image, '/top_camera/color/image_raw', self._top_rgb_cb, 10)
            self.create_subscription(Image, '/top_camera/aligned_depth_to_color/image_raw', self._top_depth_cb, 10)
            self.top_pc_pub = self.create_publisher(PointCloud2, '/top_camera/points', 10)
            self.combined_pc_pub = self.create_publisher(PointCloud2, '/camera/combined_points', 10)

            # Cache latest per-camera points for combining
            self.primary_pts = None
            self.primary_stamp = None
            self.top_pts = None
            self.top_stamp = None

            self.get_logger().info(
                f'PointCloud node started (dual camera, combined in {self.combined_frame})')
        else:
            self.get_logger().info('PointCloud node started (single camera)')

    # --- Primary camera callbacks ---

    def _primary_info_cb(self, msg):
        if self.primary.fx is not None:
            return
        self.primary.fx = msg.k[0]
        self.primary.fy = msg.k[4]
        self.primary.cx = msg.k[2]
        self.primary.cy = msg.k[5]
        self.primary.frame_id = msg.header.frame_id
        self.get_logger().info(f'Primary intrinsics: fx={self.primary.fx:.1f} fy={self.primary.fy:.1f}')

    def _primary_rgb_cb(self, msg):
        self.primary.latest_rgb = msg

    def _primary_depth_cb(self, depth_msg):
        if self.primary.fx is None or self.primary.latest_rgb is None:
            return

        pts = backproject(depth_msg, self.primary.latest_rgb, self.primary,
                          self.max_depth, self.decimation)
        if pts is None or len(pts) == 0:
            return

        # Publish individual cloud
        self.pc_pub.publish(make_pc2_msg(pts, depth_msg.header.stamp, self.primary.frame_id))

        if self.use_top_camera:
            self.primary_pts = pts
            self.primary_stamp = depth_msg.header.stamp
            self._try_publish_combined()

    # --- Top camera callbacks ---

    def _top_info_cb(self, msg):
        if self.top.fx is not None:
            return
        self.top.fx = msg.k[0]
        self.top.fy = msg.k[4]
        self.top.cx = msg.k[2]
        self.top.cy = msg.k[5]
        self.top.frame_id = msg.header.frame_id
        self.get_logger().info(f'Top intrinsics: fx={self.top.fx:.1f} fy={self.top.fy:.1f}')

    def _top_rgb_cb(self, msg):
        self.top.latest_rgb = msg

    def _top_depth_cb(self, depth_msg):
        if self.top.fx is None or self.top.latest_rgb is None:
            return

        pts = backproject(depth_msg, self.top.latest_rgb, self.top,
                          self.max_depth, self.decimation)
        if pts is None or len(pts) == 0:
            return

        # Publish individual cloud
        self.top_pc_pub.publish(make_pc2_msg(pts, depth_msg.header.stamp, self.top.frame_id))

        if self.use_top_camera:
            self.top_pts = pts
            self.top_stamp = depth_msg.header.stamp
            self._try_publish_combined()

    # --- Combined cloud ---

    def _try_publish_combined(self):
        """Transform both clouds into combined_frame and publish merged cloud."""
        if self.primary_pts is None or self.top_pts is None:
            return

        now = self.get_clock().now()

        try:
            tf_primary = self.tf_buffer.lookup_transform(
                self.combined_frame, self.primary.frame_id, rclpy.time.Time())
            tf_top = self.tf_buffer.lookup_transform(
                self.combined_frame, self.top.frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return

        primary_tf = transform_points(self.primary_pts, tf_primary)
        top_tf = transform_points(self.top_pts, tf_top)

        combined = np.concatenate([primary_tf, top_tf])
        stamp = self.primary_stamp  # use primary camera timestamp
        self.combined_pc_pub.publish(make_pc2_msg(combined, stamp, self.combined_frame))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PointCloudNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
