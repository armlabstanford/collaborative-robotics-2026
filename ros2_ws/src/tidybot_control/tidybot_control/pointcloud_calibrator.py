#!/usr/bin/env python3
"""
Interactive GUI for finding the rigid transform between two RealSense cameras.

Subscribes to both cameras' depth + RGB, backprojects into point clouds,
and provides XYZ + RPY sliders to manually align the top camera's cloud
onto the primary camera's cloud. Publishes the combined result.

Usage:
    ros2 run tidybot_control pointcloud_calibrator

Published topics:
    /camera/combined_points (PointCloud2) - merged cloud in primary camera frame
"""

import threading
import tkinter as tk
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField


PC2_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]


class CameraState:
    def __init__(self):
        self.fx = self.fy = self.cx = self.cy = None
        self.frame_id = None
        self.latest_rgb = None
        self.latest_pts = None


def backproject(depth_msg, rgb_msg, cam, max_depth, decimation):
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


def rpy_to_rotation_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ], dtype=np.float32)
    return R


def transform_points(pts, tx, ty, tz, roll, pitch, yaw):
    R = rpy_to_rotation_matrix(roll, pitch, yaw)
    tvec = np.array([tx, ty, tz], dtype=np.float32)
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1)
    xyz_tf = xyz @ R.T + tvec
    out = np.copy(pts)
    out['x'] = xyz_tf[:, 0]
    out['y'] = xyz_tf[:, 1]
    out['z'] = xyz_tf[:, 2]
    return out


def make_pc2_msg(pts, stamp, frame_id):
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


class PointCloudCalibrator(Node):
    def __init__(self):
        super().__init__('pointcloud_calibrator')

        self.declare_parameter('max_depth_m', 5.0)
        self.declare_parameter('decimation', 4)

        self.max_depth = self.get_parameter('max_depth_m').value
        self.decimation = self.get_parameter('decimation').value

        self.primary = CameraState()
        self.top = CameraState()

        # Primary camera
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._pri_info, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self._pri_rgb, 10)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self._pri_depth, 10)

        # Top camera
        self.create_subscription(CameraInfo, '/top_camera/color/camera_info', self._top_info, 10)
        self.create_subscription(Image, '/top_camera/color/image_raw', self._top_rgb, 10)
        self.create_subscription(Image, '/top_camera/aligned_depth_to_color/image_raw', self._top_depth, 10)

        # Publishers
        self.combined_pub = self.create_publisher(PointCloud2, '/camera/combined_points', 10)
        self.primary_pub = self.create_publisher(PointCloud2, '/camera/points', 10)
        self.top_pub = self.create_publisher(PointCloud2, '/top_camera/points', 10)

        # Transform values (updated by GUI)
        self.tx = 0.0
        self.ty = 0.0
        self.tz = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.lock = threading.Lock()

        # Publish combined cloud at 10 Hz
        self.create_timer(0.1, self._publish_combined)

        self.get_logger().info('PointCloud Calibrator started — adjust sliders to align cameras')

    # --- Primary camera ---
    def _pri_info(self, msg):
        if self.primary.fx is not None:
            return
        self.primary.fx, self.primary.fy = msg.k[0], msg.k[4]
        self.primary.cx, self.primary.cy = msg.k[2], msg.k[5]
        self.primary.frame_id = msg.header.frame_id
        self.get_logger().info(f'Primary intrinsics: fx={self.primary.fx:.1f}')

    def _pri_rgb(self, msg):
        self.primary.latest_rgb = msg

    def _pri_depth(self, msg):
        if self.primary.fx is None or self.primary.latest_rgb is None:
            return
        pts = backproject(msg, self.primary.latest_rgb, self.primary,
                          self.max_depth, self.decimation)
        if pts is not None and len(pts) > 0:
            self.primary.latest_pts = (pts, msg.header.stamp)
            self.primary_pub.publish(make_pc2_msg(pts, msg.header.stamp, self.primary.frame_id))

    # --- Top camera ---
    def _top_info(self, msg):
        if self.top.fx is not None:
            return
        self.top.fx, self.top.fy = msg.k[0], msg.k[4]
        self.top.cx, self.top.cy = msg.k[2], msg.k[5]
        self.top.frame_id = msg.header.frame_id
        self.get_logger().info(f'Top intrinsics: fx={self.top.fx:.1f}')

    def _top_rgb(self, msg):
        self.top.latest_rgb = msg

    def _top_depth(self, msg):
        if self.top.fx is None or self.top.latest_rgb is None:
            return
        pts = backproject(msg, self.top.latest_rgb, self.top,
                          self.max_depth, self.decimation)
        if pts is not None and len(pts) > 0:
            self.top.latest_pts = (pts, msg.header.stamp)
            self.top_pub.publish(make_pc2_msg(pts, msg.header.stamp, self.top.frame_id))

    # --- Combined ---
    def _publish_combined(self):
        if self.primary.latest_pts is None or self.top.latest_pts is None:
            return

        pri_pts, stamp = self.primary.latest_pts
        top_pts, _ = self.top.latest_pts

        with self.lock:
            tx, ty, tz = self.tx, self.ty, self.tz
            roll, pitch, yaw = self.roll, self.pitch, self.yaw

        top_tf = transform_points(top_pts, tx, ty, tz, roll, pitch, yaw)
        combined = np.concatenate([pri_pts, top_tf])
        self.combined_pub.publish(
            make_pc2_msg(combined, stamp, self.primary.frame_id))


def build_gui(node):
    """Build tkinter slider GUI. Runs in main thread."""
    root = tk.Tk()
    root.title('Point Cloud Calibrator — Top Camera Transform')
    root.geometry('500x480')
    root.configure(bg='#2b2b2b')

    style_label = {'bg': '#2b2b2b', 'fg': '#ffffff', 'font': ('monospace', 11)}
    style_value = {'bg': '#2b2b2b', 'fg': '#00ff88', 'font': ('monospace', 11, 'bold')}

    sliders = {}

    def make_slider(parent, label, from_, to, resolution, row, attr):
        tk.Label(parent, text=label, **style_label).grid(
            row=row, column=0, sticky='w', padx=10, pady=2)

        val_label = tk.Label(parent, text='0.000', width=8, anchor='e', **style_value)
        val_label.grid(row=row, column=2, padx=10, pady=2)

        def on_change(val):
            v = float(val)
            with node.lock:
                setattr(node, attr, v)
            val_label.config(text=f'{v:.3f}')

        s = tk.Scale(parent, from_=from_, to=to, resolution=resolution,
                     orient=tk.HORIZONTAL, length=280, command=on_change,
                     bg='#3c3c3c', fg='#ffffff', troughcolor='#555555',
                     highlightthickness=0, font=('monospace', 9))
        s.set(0.0)
        s.grid(row=row, column=1, padx=5, pady=2)
        sliders[attr] = s

    frame = tk.Frame(root, bg='#2b2b2b')
    frame.pack(fill='both', expand=True, padx=10, pady=10)

    tk.Label(frame, text='=== Translation (meters) ===',
             bg='#2b2b2b', fg='#88aaff', font=('monospace', 12, 'bold')).grid(
        row=0, column=0, columnspan=3, pady=(5, 8))

    make_slider(frame, 'X', -2.0, 2.0, 0.005, 1, 'tx')
    make_slider(frame, 'Y', -2.0, 2.0, 0.005, 2, 'ty')
    make_slider(frame, 'Z', -2.0, 2.0, 0.005, 3, 'tz')

    tk.Label(frame, text='=== Rotation (radians) ===',
             bg='#2b2b2b', fg='#88aaff', font=('monospace', 12, 'bold')).grid(
        row=4, column=0, columnspan=3, pady=(15, 8))

    make_slider(frame, 'Roll',  -3.14, 3.14, 0.01, 5, 'roll')
    make_slider(frame, 'Pitch', -3.14, 3.14, 0.01, 6, 'pitch')
    make_slider(frame, 'Yaw',   -3.14, 3.14, 0.01, 7, 'yaw')

    # Print button
    def print_transform():
        with node.lock:
            print(f'\n--- Current Transform (top_camera -> primary_camera) ---')
            print(f'  x: {node.tx:.4f}  y: {node.ty:.4f}  z: {node.tz:.4f}')
            print(f'  roll: {node.roll:.4f}  pitch: {node.pitch:.4f}  yaw: {node.yaw:.4f}')
            print(f'---')

    def reset_all():
        for attr, s in sliders.items():
            s.set(0.0)

    btn_frame = tk.Frame(root, bg='#2b2b2b')
    btn_frame.pack(fill='x', padx=10, pady=(0, 10))

    tk.Button(btn_frame, text='Print Transform', command=print_transform,
              bg='#4a4a4a', fg='#ffffff', font=('monospace', 10),
              relief='flat', padx=15, pady=5).pack(side='left', padx=5)

    tk.Button(btn_frame, text='Reset', command=reset_all,
              bg='#4a4a4a', fg='#ffffff', font=('monospace', 10),
              relief='flat', padx=15, pady=5).pack(side='left', padx=5)

    return root


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCalibrator()

    # Run ROS spinning in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run GUI in main thread (tkinter requirement)
    root = build_gui(node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
