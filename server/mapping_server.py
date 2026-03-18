#!/usr/bin/env python3
"""
Dense Mapping Server — receives RGB+Depth+Pose from the robot via ZMQ and
builds a GPU-accelerated TSDF map using Open3D, with web-based 3D
visualization via viser (accessible from any browser, works over SSH).

Requirements:
    pip install open3d pyzmq msgpack numpy lz4 opencv-python viser trimesh

Usage:
    python mapping_server.py --robot-ip <ROBOT_IP>
    python mapping_server.py --robot-ip 10.0.0.2 --voxel-size 0.01
    python mapping_server.py --robot-ip 10.0.0.2 --use-icp

    Then open http://<SERVER_IP>:8080 in a browser.

GUI controls (in browser):
    Save Map       — save current map to disk (tsdf_map/)
    Load Map       — load previously saved map
    Reset Map      — clear the TSDF volume
    Show Mesh      — toggle mesh vs point cloud
    Point Size     — adjust point cloud point size
    Update Interval — seconds between vis refreshes
"""

import argparse
import os
import time
import threading

import numpy as np
import cv2
import zmq
import msgpack
import lz4.frame
import open3d as o3d
import open3d.core as o3c
import trimesh
import viser
import viser.transforms as vtf


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def decode_frame(packed: bytes) -> dict:
    """Unpack a ZMQ frame from the bridge node."""
    frame = msgpack.unpackb(packed, raw=False)

    # Decode JPEG → RGB numpy
    rgb_arr = np.frombuffer(frame['rgb_jpeg'], dtype=np.uint8)
    bgr = cv2.imdecode(rgb_arr, cv2.IMREAD_COLOR)
    frame['rgb'] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    # Decode depth
    h, w = frame['depth_shape']
    depth_bytes = frame['depth']
    if frame.get('depth_lz4', False):
        depth_bytes = lz4.frame.decompress(depth_bytes)
    frame['depth_u16'] = np.frombuffer(depth_bytes, dtype=np.uint16).reshape(h, w)
    frame['depth_m'] = frame['depth_u16'].astype(np.float32) * 0.001

    frame['camera_to_world'] = np.array(frame['camera_to_world'], dtype=np.float64)
    frame['intrinsics'] = list(frame['intrinsics'])
    return frame


def make_o3d_intrinsic(intrinsics, width, height):
    """Create Open3D camera intrinsic from [fx, fy, cx, cy]."""
    fx, fy, cx, cy = intrinsics
    return o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)


def rotation_matrix_to_wxyz(R):
    """Convert 3x3 rotation matrix to wxyz quaternion."""
    m = np.asarray(R, dtype=np.float64)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])


# ---------------------------------------------------------------------------
# TSDF Mapper
# ---------------------------------------------------------------------------

class TSDFMapper:
    def __init__(self, voxel_size=0.02, block_resolution=16,
                 device_str='CUDA:0', use_icp=False):
        self.voxel_size = voxel_size
        self.use_icp = use_icp

        # Try CUDA, fall back to CPU
        if 'CUDA' in device_str:
            try:
                self.device = o3c.Device(device_str)
                _ = o3c.Tensor([0], device=self.device)
            except RuntimeError:
                print(f'[WARN] {device_str} not available, falling back to CPU')
                self.device = o3c.Device('CPU:0')
        else:
            self.device = o3c.Device(device_str)

        print(f'[INFO] Using device: {self.device}')

        self.block_resolution = block_resolution
        self.voxel_block_grid = None
        self._init_volume()

        # Pose history for trajectory visualization
        self.poses = []            # list of (3,) positions
        self.pose_matrices = []    # list of (4,4) full transforms
        self.frame_count = 0
        self.lock = threading.Lock()

        # Latest camera image for frustum display
        self.latest_rgb = None

        # ICP state
        self.prev_pose = None

    def _init_volume(self):
        """Create a fresh TSDF voxel block grid."""
        self.voxel_block_grid = o3d.t.geometry.VoxelBlockGrid(
            attr_names=('tsdf', 'weight', 'color'),
            attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
            attr_channels=((1,), (1,), (3,)),
            voxel_size=self.voxel_size,
            block_resolution=self.block_resolution,
            block_count=50000,
            device=self.device,
        )
        self.poses = []
        self.pose_matrices = []
        self.frame_count = 0
        self.latest_rgb = None
        self.prev_pose = None

    def reset(self):
        with self.lock:
            print('[INFO] Resetting map')
            self._init_volume()

    def integrate(self, frame: dict):
        """Integrate one frame into the TSDF volume."""
        h, w = frame['depth_shape']
        intrinsic = make_o3d_intrinsic(frame['intrinsics'], w, h)
        intrinsic_t = o3c.Tensor(intrinsic.intrinsic_matrix, dtype=o3c.float64)

        depth_t = o3d.t.geometry.Image(
            o3c.Tensor(frame['depth_m'], dtype=o3c.float32))
        color_t = o3d.t.geometry.Image(
            o3c.Tensor(frame['rgb'].astype(np.float32) / 255.0, dtype=o3c.float32))

        cam2world = frame['camera_to_world']

        # Optional ICP refinement
        if self.use_icp and self.prev_pose is not None:
            cam2world = self._refine_icp(depth_t, intrinsic_t, cam2world)

        extrinsic = np.linalg.inv(cam2world)
        extrinsic_t = o3c.Tensor(extrinsic, dtype=o3c.float64)

        with self.lock:
            frustum_hash = self.voxel_block_grid.compute_unique_block_coordinates(
                depth_t, intrinsic_t, extrinsic_t,
                depth_scale=1.0, depth_max=5.0)

            self.voxel_block_grid.integrate(
                frustum_hash, depth_t, color_t,
                intrinsic_t, extrinsic_t,
                depth_scale=1.0, depth_max=5.0)

            self.poses.append(cam2world[:3, 3].copy())
            self.pose_matrices.append(cam2world.copy())
            self.frame_count += 1
            self.latest_rgb = frame['rgb']

        self.prev_pose = cam2world.copy()

    def _refine_icp(self, depth_t, intrinsic_t, init_pose):
        """Point-to-plane ICP against the existing TSDF for drift correction."""
        try:
            with self.lock:
                pcd = self.voxel_block_grid.extract_point_cloud(weight_threshold=3.0)
            if pcd.is_empty():
                return init_pose

            h = depth_t.as_tensor().shape[0]
            w = depth_t.as_tensor().shape[1]
            intrinsic_legacy = o3d.camera.PinholeCameraIntrinsic(
                w, h,
                intrinsic_t[0, 0].item(), intrinsic_t[1, 1].item(),
                intrinsic_t[0, 2].item(), intrinsic_t[1, 2].item())

            depth_legacy = o3d.geometry.Image(
                np.ascontiguousarray(depth_t.as_tensor().cpu().numpy()))
            source_pcd = o3d.geometry.PointCloud.create_from_depth_image(
                depth_legacy, intrinsic_legacy, np.linalg.inv(init_pose),
                depth_scale=1.0, depth_trunc=5.0)

            if len(source_pcd.points) < 100:
                return init_pose

            target_pcd = pcd.to_legacy()
            target_pcd.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

            result = o3d.pipelines.registration.registration_icp(
                source_pcd, target_pcd, 0.05, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20))

            if result.fitness > 0.3:
                refined = result.transformation @ np.linalg.inv(
                    np.linalg.inv(init_pose))
                return refined
        except Exception as e:
            print(f'[WARN] ICP failed: {e}')

        return init_pose

    def extract_mesh_arrays(self):
        """Extract mesh vertices, faces, colors as numpy arrays."""
        with self.lock:
            try:
                mesh_t = self.voxel_block_grid.extract_triangle_mesh(
                    weight_threshold=3.0)
                mesh = mesh_t.to_legacy()
            except Exception:
                return None, None, None

        if len(mesh.vertices) == 0:
            return None, None, None

        vertices = np.asarray(mesh.vertices).astype(np.float32)
        faces = np.asarray(mesh.triangles).astype(np.int32)
        if mesh.has_vertex_colors():
            colors = (np.asarray(mesh.vertex_colors) * 255).clip(0, 255).astype(np.uint8)
        else:
            colors = np.full((len(vertices), 3), 180, dtype=np.uint8)
        return vertices, faces, colors

    def extract_pointcloud_arrays(self):
        """Extract point cloud positions and colors as numpy arrays."""
        with self.lock:
            try:
                pcd_t = self.voxel_block_grid.extract_point_cloud(
                    weight_threshold=3.0)
                pcd = pcd_t.to_legacy()
            except Exception:
                return None, None

        if len(pcd.points) == 0:
            return None, None

        points = np.asarray(pcd.points).astype(np.float32)
        if pcd.has_colors():
            colors = (np.asarray(pcd.colors) * 255).clip(0, 255).astype(np.uint8)
        else:
            colors = np.full((len(points), 3), 180, dtype=np.uint8)
        return points, colors

    def save(self, directory='tsdf_map'):
        """Save the TSDF volume and poses to disk."""
        os.makedirs(directory, exist_ok=True)
        with self.lock:
            try:
                mesh_t = self.voxel_block_grid.extract_triangle_mesh(
                    weight_threshold=3.0)
                mesh = mesh_t.to_legacy()
                if len(mesh.vertices) > 0:
                    o3d.io.write_triangle_mesh(
                        os.path.join(directory, 'mesh.ply'), mesh)
            except Exception:
                pass
            try:
                pcd_t = self.voxel_block_grid.extract_point_cloud(
                    weight_threshold=3.0)
                pcd = pcd_t.to_legacy()
                if len(pcd.points) > 0:
                    o3d.io.write_point_cloud(
                        os.path.join(directory, 'pointcloud.ply'), pcd)
            except Exception:
                pass
            if self.poses:
                np.save(os.path.join(directory, 'poses.npy'),
                        np.array(self.poses))
        print(f'[INFO] Map saved to {directory}/')

    def load_mesh_arrays(self, directory='tsdf_map'):
        """Load a previously saved mesh."""
        mesh_path = os.path.join(directory, 'mesh.ply')
        if not os.path.exists(mesh_path):
            return None, None, None
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        if len(mesh.vertices) == 0:
            return None, None, None
        vertices = np.asarray(mesh.vertices).astype(np.float32)
        faces = np.asarray(mesh.triangles).astype(np.int32)
        if mesh.has_vertex_colors():
            colors = (np.asarray(mesh.vertex_colors) * 255).clip(0, 255).astype(np.uint8)
        else:
            colors = np.full((len(vertices), 3), 180, dtype=np.uint8)
        print(f'[INFO] Loaded mesh: {len(vertices)} vertices')
        return vertices, faces, colors


# ---------------------------------------------------------------------------
# Viser web visualizer
# ---------------------------------------------------------------------------

class ViserVisualizer:
    def __init__(self, mapper: TSDFMapper, vis_port: int = 8080):
        self.mapper = mapper
        self.running = True
        self.show_mesh = True
        self.update_interval = 2.0
        self.point_size = 0.01

        self.server = viser.ViserServer(host='0.0.0.0', port=vis_port)
        print(f'[INFO] Viser web UI at http://0.0.0.0:{vis_port}')

        # Add world frame
        self.server.scene.add_frame('/world', axes_length=0.3, axes_radius=0.01)

        # Add ground grid
        self.server.scene.add_grid('/grid', width=10.0, height=10.0,
                                   cell_size=0.5, position=(0, 0, 0))

        # GUI controls
        with self.server.gui.add_folder('Map Controls'):
            self._save_btn = self.server.gui.add_button('Save Map')
            self._load_btn = self.server.gui.add_button('Load Map')
            self._reset_btn = self.server.gui.add_button('Reset Map')

        with self.server.gui.add_folder('Display'):
            self._show_mesh = self.server.gui.add_checkbox('Show Mesh', initial_value=True)
            self._point_size = self.server.gui.add_slider(
                'Point Size', min=0.001, max=0.05, step=0.001, initial_value=0.01)
            self._update_interval = self.server.gui.add_slider(
                'Update Interval (s)', min=0.5, max=10.0, step=0.5, initial_value=2.0)

        with self.server.gui.add_folder('Status'):
            self._status_text = self.server.gui.add_text('Frames', initial_value='0', disabled=True)
            self._fps_text = self.server.gui.add_text('FPS', initial_value='--', disabled=True)

        # Wire up callbacks
        self._save_btn.on_click(self._on_save)
        self._load_btn.on_click(self._on_load)
        self._reset_btn.on_click(self._on_reset)
        self._show_mesh.on_update(self._on_display_change)
        self._point_size.on_update(self._on_display_change)
        self._update_interval.on_update(self._on_interval_change)

    def _on_save(self, _):
        self.mapper.save()
        print('[INFO] Map saved via GUI')

    def _on_load(self, _):
        verts, faces, colors = self.mapper.load_mesh_arrays()
        if verts is not None:
            mesh = trimesh.Trimesh(
                vertices=verts, faces=faces,
                vertex_colors=colors)
            self.server.scene.add_mesh_trimesh('/map/geometry', mesh=mesh)
            print('[INFO] Map loaded via GUI')

    def _on_reset(self, _):
        self.mapper.reset()
        self.server.scene.remove('/map')
        self.server.scene.remove('/trajectory')
        self.server.scene.remove('/camera')
        print('[INFO] Map reset via GUI')

    def _on_display_change(self, _):
        self.show_mesh = self._show_mesh.value
        self.point_size = self._point_size.value
        self._force_update = True

    def _on_interval_change(self, _):
        self.update_interval = self._update_interval.value

    def update_scene(self):
        """Extract geometry from TSDF and push to viser scene."""
        if self.show_mesh:
            verts, faces, colors = self.mapper.extract_mesh_arrays()
            if verts is not None:
                mesh = trimesh.Trimesh(
                    vertices=verts, faces=faces,
                    vertex_colors=colors)
                self.server.scene.add_mesh_trimesh(
                    '/map/geometry', mesh=mesh)
        else:
            points, colors = self.mapper.extract_pointcloud_arrays()
            if points is not None:
                self.server.scene.add_point_cloud(
                    '/map/geometry', points=points, colors=colors,
                    point_size=self.point_size)

        # Trajectory as point cloud (red dots at each pose)
        with self.mapper.lock:
            poses = list(self.mapper.poses)
            pose_matrices = list(self.mapper.pose_matrices)

        if len(poses) >= 2:
            traj_pts = np.array(poses, dtype=np.float32)
            traj_colors = np.zeros((len(traj_pts), 3), dtype=np.uint8)
            traj_colors[:, 0] = 255  # red
            self.server.scene.add_point_cloud(
                '/trajectory/points', points=traj_pts, colors=traj_colors,
                point_size=0.02)

        # Current camera frustum with image
        if pose_matrices:
            T = pose_matrices[-1]
            pos = T[:3, 3]
            wxyz = rotation_matrix_to_wxyz(T[:3, :3])

            self.server.scene.add_frame(
                '/camera/axes', axes_length=0.1, axes_radius=0.005,
                wxyz=wxyz, position=pos)

            rgb = self.mapper.latest_rgb
            if rgb is not None:
                h, w = rgb.shape[:2]
                self.server.scene.add_camera_frustum(
                    '/camera/frustum',
                    fov=np.deg2rad(42),
                    aspect=w / h,
                    scale=0.15,
                    image=rgb,
                    wxyz=wxyz,
                    position=pos,
                )

    def run(self):
        """Main visualization loop."""
        self._force_update = False
        last_update = 0.0
        last_count = 0

        while self.running:
            time.sleep(0.1)

            now = time.time()
            count = self.mapper.frame_count

            # Update status text
            self._status_text.value = str(count)

            need_update = (
                self._force_update or
                (count > last_count and now - last_update >= self.update_interval)
            )

            if need_update and count > 0:
                self._force_update = False
                last_update = now
                t0 = time.time()
                self.update_scene()
                dt = time.time() - t0
                self._fps_text.value = f'{1.0/dt:.1f} extract/s' if dt > 0 else '--'
                last_count = count


# ---------------------------------------------------------------------------
# Receiver thread
# ---------------------------------------------------------------------------

def receiver_loop(robot_ip, port, mapper: TSDFMapper, running_flag):
    """Background thread — receives ZMQ frames and integrates into TSDF."""
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b'')
    sub.setsockopt(zmq.RCVHWM, 2)
    sub.setsockopt(zmq.CONFLATE, 1)  # only keep latest message
    addr = f'tcp://{robot_ip}:{port}'
    sub.connect(addr)
    print(f'[INFO] ZMQ SUB connected to {addr}')

    fps_counter = 0
    fps_time = time.time()

    while running_flag():
        try:
            packed = sub.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            time.sleep(0.005)
            continue

        frame = decode_frame(packed)
        mapper.integrate(frame)

        fps_counter += 1
        now = time.time()
        if now - fps_time >= 5.0:
            fps = fps_counter / (now - fps_time)
            print(f'[INFO] Integration: {fps:.1f} fps | '
                  f'{mapper.frame_count} total frames')
            fps_counter = 0
            fps_time = now

    sub.close()
    ctx.term()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Dense mapping server for TidyBot2')
    parser.add_argument('--robot-ip', required=True,
                        help='IP of the robot (ZMQ publisher)')
    parser.add_argument('--port', type=int, default=5555,
                        help='ZMQ port (default: 5555)')
    parser.add_argument('--voxel-size', type=float, default=0.02,
                        help='TSDF voxel size in meters (default: 0.02)')
    parser.add_argument('--device', default='CUDA:0',
                        help='Open3D device (default: CUDA:0)')
    parser.add_argument('--use-icp', action='store_true',
                        help='Enable ICP refinement against TSDF')
    parser.add_argument('--vis-port', type=int, default=8080,
                        help='Viser web UI port (default: 8080)')
    args = parser.parse_args()

    mapper = TSDFMapper(
        voxel_size=args.voxel_size,
        device_str=args.device,
        use_icp=args.use_icp,
    )

    visualizer = ViserVisualizer(mapper, vis_port=args.vis_port)

    recv_thread = threading.Thread(
        target=receiver_loop,
        args=(args.robot_ip, args.port, mapper, lambda: visualizer.running),
        daemon=True)
    recv_thread.start()

    print(f'[INFO] Open http://0.0.0.0:{args.vis_port} in your browser')

    try:
        visualizer.run()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.running = False
        recv_thread.join(timeout=2.0)
        print(f'\n[INFO] Saving map ({mapper.frame_count} frames)...')
        mapper.save()
        print('[INFO] Done.')


if __name__ == '__main__':
    main()
