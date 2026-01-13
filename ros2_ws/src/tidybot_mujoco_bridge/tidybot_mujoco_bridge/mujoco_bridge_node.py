#!/usr/bin/env python3
"""
MuJoCo Bridge Node for TidyBot2.

This node wraps MuJoCo simulation and exposes ROS2 topics for:
- Joint states (publish)
- Camera images (publish)
- Odometry (publish)
- TF transforms (publish)
- Joint commands (subscribe)
- Base velocity commands (subscribe)
"""

import os
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Header
from tf2_ros import TransformBroadcaster

import mujoco
import mujoco.viewer

# Try to import cv_bridge for image publishing
try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


class MuJoCoBridgeNode(Node):
    """ROS2 node that bridges MuJoCo simulation to ROS2 topics."""

    # Joint name mapping (order matches MuJoCo actuator order)
    JOINT_NAMES = [
        # Base (3 DOF) - virtual joints, not published
        # 'joint_x', 'joint_y', 'joint_th',
        # Pan-tilt (2 DOF)
        'camera_pan', 'camera_tilt',
        # Right arm (5 DOF)
        'right_waist', 'right_shoulder', 'right_elbow', 'right_wrist_angle', 'right_wrist_rotate',
        # Right gripper (2 DOF)
        'right_left_finger', 'right_right_finger',
        # Left arm (5 DOF)
        'left_waist', 'left_shoulder', 'left_elbow', 'left_wrist_angle', 'left_wrist_rotate',
        # Left gripper (2 DOF)
        'left_left_finger', 'left_right_finger',
    ]

    # Actuator names in MuJoCo (same order as JOINT_NAMES but includes base)
    ACTUATOR_NAMES = [
        'joint_x', 'joint_y', 'joint_th',
        'camera_pan', 'camera_tilt',
        'right_waist', 'right_shoulder', 'right_elbow', 'right_wrist_angle', 'right_wrist_rotate',
        'right_left_finger', 'right_right_finger',
        'left_waist', 'left_shoulder', 'left_elbow', 'left_wrist_angle', 'left_wrist_rotate',
        'left_left_finger', 'left_right_finger',
    ]

    def __init__(self):
        super().__init__('mujoco_bridge')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('sim_rate', 500.0)  # Hz
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('camera_rate', 30.0)  # Hz
        self.declare_parameter('show_viewer', True)

        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.sim_rate = self.get_parameter('sim_rate').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.camera_rate = self.get_parameter('camera_rate').get_parameter_value().double_value
        show_viewer = self.get_parameter('show_viewer').get_parameter_value().bool_value

        # Find model path if not specified
        if not model_path:
            # Default to the simulation assets
            # When installed, we need to find the repo root from a known location
            # Try environment variable first, then fall back to searching
            repo_root = os.environ.get('TIDYBOT_REPO_ROOT')
            if not repo_root:
                # Search upward for the simulation directory
                search_path = os.path.dirname(os.path.abspath(__file__))
                for _ in range(10):  # Max 10 levels up
                    candidate = os.path.join(search_path, 'simulation', 'assets', 'mujoco')
                    if os.path.isdir(candidate):
                        repo_root = search_path
                        break
                    search_path = os.path.dirname(search_path)

            if repo_root:
                model_path = os.path.join(
                    repo_root, 'simulation', 'assets', 'mujoco', 'scene_wx200_bimanual.xml'
                )
            else:
                # Last resort: assume we're in ros2_ws inside the repo
                model_path = os.path.expanduser(
                    '~/Documents/collaborative-robotics-2026/simulation/assets/mujoco/scene_wx200_bimanual.xml'
                )

        self.get_logger().info(f'Loading MuJoCo model from: {model_path}')

        # Load MuJoCo model
        try:
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
        except Exception as e:
            self.get_logger().error(f'Failed to load MuJoCo model: {e}')
            raise

        # Build actuator ID lookup
        self.actuator_ids = {}
        for name in self.ACTUATOR_NAMES:
            try:
                self.actuator_ids[name] = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
                )
            except Exception:
                self.get_logger().warn(f'Actuator not found: {name}')

        # Build joint ID lookup
        self.joint_ids = {}
        for name in self.JOINT_NAMES:
            try:
                self.joint_ids[name] = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, name
                )
            except Exception:
                self.get_logger().warn(f'Joint not found: {name}')

        # Image bridge
        if HAS_CV_BRIDGE:
            self.cv_bridge = CvBridge()
        else:
            self.get_logger().warn('cv_bridge not available, camera images disabled')

        # Create MuJoCo renderer for camera images
        self.renderer = mujoco.Renderer(self.model, height=480, width=640)

        # State variables
        self.sim_step_count = 0
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_th = 0.0
        self.cmd_vel = Twist()
        self.current_vel = Twist()  # Smoothed velocity
        self.last_sim_time = None
        self.lock = threading.Lock()

        # Acceleration limits for smooth motion (m/s^2 and rad/s^2)
        self.max_linear_accel = 1.0   # m/s^2
        self.max_angular_accel = 2.0  # rad/s^2

        # Target positions for all actuators (initialized to current)
        self.target_ctrl = np.zeros(self.model.nu)

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', qos)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.right_arm_sub = self.create_subscription(
            Float64MultiArray, '/right_arm/joint_cmd', self.right_arm_callback, 10
        )
        self.left_arm_sub = self.create_subscription(
            Float64MultiArray, '/left_arm/joint_cmd', self.left_arm_callback, 10
        )
        self.right_gripper_sub = self.create_subscription(
            Float64MultiArray, '/right_gripper/cmd', self.right_gripper_callback, 10
        )
        self.left_gripper_sub = self.create_subscription(
            Float64MultiArray, '/left_gripper/cmd', self.left_gripper_callback, 10
        )
        self.pan_tilt_sub = self.create_subscription(
            Float64MultiArray, '/camera/pan_tilt_cmd', self.pan_tilt_callback, 10
        )

        # Timers
        sim_period = 1.0 / self.sim_rate
        self.sim_timer = self.create_timer(sim_period, self.sim_step_callback)

        publish_period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(publish_period, self.publish_callback)

        camera_period = 1.0 / self.camera_rate
        self.camera_timer = self.create_timer(camera_period, self.camera_callback)

        # Launch viewer in separate thread if requested
        self.viewer = None
        if show_viewer:
            self.viewer_thread = threading.Thread(target=self.run_viewer, daemon=True)
            self.viewer_thread.start()

        self.get_logger().info('MuJoCo Bridge initialized successfully')
        self.get_logger().info(f'  Sim rate: {self.sim_rate} Hz')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Camera rate: {self.camera_rate} Hz')

    def run_viewer(self):
        """Run MuJoCo viewer in a separate thread."""
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.viewer = viewer
            while viewer.is_running():
                with self.lock:
                    viewer.sync()

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands for the mobile base."""
        with self.lock:
            self.cmd_vel = msg

    def right_arm_callback(self, msg: Float64MultiArray):
        """Handle joint commands for right arm (5 joints)."""
        if len(msg.data) != 5:
            self.get_logger().warn(f'Right arm command has {len(msg.data)} values, expected 5')
            return
        with self.lock:
            for i, name in enumerate(['right_waist', 'right_shoulder', 'right_elbow',
                                       'right_wrist_angle', 'right_wrist_rotate']):
                if name in self.actuator_ids:
                    self.target_ctrl[self.actuator_ids[name]] = msg.data[i]

    def left_arm_callback(self, msg: Float64MultiArray):
        """Handle joint commands for left arm (5 joints)."""
        if len(msg.data) != 5:
            self.get_logger().warn(f'Left arm command has {len(msg.data)} values, expected 5')
            return
        with self.lock:
            for i, name in enumerate(['left_waist', 'left_shoulder', 'left_elbow',
                                       'left_wrist_angle', 'left_wrist_rotate']):
                if name in self.actuator_ids:
                    self.target_ctrl[self.actuator_ids[name]] = msg.data[i]

    def right_gripper_callback(self, msg: Float64MultiArray):
        """Handle gripper command for right gripper (normalized 0-1)."""
        if len(msg.data) < 1:
            return
        # Convert normalized position to finger position
        pos = msg.data[0] * 0.022  # 0.022m is max finger travel
        with self.lock:
            if 'right_left_finger' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['right_left_finger']] = pos
            if 'right_right_finger' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['right_right_finger']] = pos

    def left_gripper_callback(self, msg: Float64MultiArray):
        """Handle gripper command for left gripper (normalized 0-1)."""
        if len(msg.data) < 1:
            return
        pos = msg.data[0] * 0.022
        with self.lock:
            if 'left_left_finger' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['left_left_finger']] = pos
            if 'left_right_finger' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['left_right_finger']] = pos

    def pan_tilt_callback(self, msg: Float64MultiArray):
        """Handle pan-tilt camera commands."""
        if len(msg.data) < 2:
            return
        with self.lock:
            if 'camera_pan' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['camera_pan']] = msg.data[0]
            if 'camera_tilt' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['camera_tilt']] = msg.data[1]

    def sim_step_callback(self):
        """Step the MuJoCo simulation."""
        now = time.perf_counter()

        # Use actual elapsed time for accurate integration
        if self.last_sim_time is None:
            dt = 1.0 / self.sim_rate
        else:
            dt = now - self.last_sim_time
            # Clamp dt to avoid huge jumps if callback was delayed
            dt = min(dt, 2.0 / self.sim_rate)
        self.last_sim_time = now

        with self.lock:
            # Smooth velocity commands with acceleration limits
            target_vx = self.cmd_vel.linear.x
            target_vy = self.cmd_vel.linear.y
            target_vth = self.cmd_vel.angular.z

            # Apply acceleration limiting for smooth motion
            max_dv_linear = self.max_linear_accel * dt
            max_dv_angular = self.max_angular_accel * dt

            # Ramp linear.x
            dv = target_vx - self.current_vel.linear.x
            if abs(dv) > max_dv_linear:
                dv = max_dv_linear if dv > 0 else -max_dv_linear
            self.current_vel.linear.x += dv

            # Ramp linear.y
            dv = target_vy - self.current_vel.linear.y
            if abs(dv) > max_dv_linear:
                dv = max_dv_linear if dv > 0 else -max_dv_linear
            self.current_vel.linear.y += dv

            # Ramp angular.z
            dv = target_vth - self.current_vel.angular.z
            if abs(dv) > max_dv_angular:
                dv = max_dv_angular if dv > 0 else -max_dv_angular
            self.current_vel.angular.z += dv

            # Use smoothed velocities for integration
            vx = self.current_vel.linear.x
            vy = self.current_vel.linear.y
            vth = self.current_vel.angular.z

            # Transform velocities from robot frame to world frame
            cos_th = np.cos(self.base_th)
            sin_th = np.sin(self.base_th)
            self.base_x += (vx * cos_th - vy * sin_th) * dt
            self.base_y += (vx * sin_th + vy * cos_th) * dt
            self.base_th += vth * dt

            # Set base position actuators
            if 'joint_x' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['joint_x']] = self.base_x
            if 'joint_y' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['joint_y']] = self.base_y
            if 'joint_th' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['joint_th']] = self.base_th

            # Apply control targets
            self.data.ctrl[:] = self.target_ctrl

            # Step simulation
            mujoco.mj_step(self.model, self.data)

        self.sim_step_count += 1

    def publish_callback(self):
        """Publish joint states, odometry, and TF."""
        now = self.get_clock().now().to_msg()

        with self.lock:
            # Publish joint states
            joint_state = JointState()
            joint_state.header.stamp = now
            joint_state.header.frame_id = 'base_link'

            for name in self.JOINT_NAMES:
                if name in self.joint_ids:
                    joint_id = self.joint_ids[name]
                    qpos_adr = self.model.jnt_qposadr[joint_id]
                    qvel_adr = self.model.jnt_dofadr[joint_id]

                    joint_state.name.append(name)
                    joint_state.position.append(float(self.data.qpos[qpos_adr]))
                    joint_state.velocity.append(float(self.data.qvel[qvel_adr]))
                    joint_state.effort.append(0.0)  # MuJoCo doesn't directly expose this

            self.joint_state_pub.publish(joint_state)

            # Publish odometry
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = self.base_x
            odom.pose.pose.position.y = self.base_y
            odom.pose.pose.position.z = 0.0

            # Convert yaw to quaternion
            cy = np.cos(self.base_th * 0.5)
            sy = np.sin(self.base_th * 0.5)
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = sy
            odom.pose.pose.orientation.w = cy

            odom.twist.twist = self.current_vel  # Use smoothed velocity

            self.odom_pub.publish(odom)

            # Publish TF: odom -> base_link
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.base_x
            t.transform.translation.y = self.base_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = sy
            t.transform.rotation.w = cy

            self.tf_broadcaster.sendTransform(t)

    def camera_callback(self):
        """Publish camera images."""
        if not HAS_CV_BRIDGE:
            return

        now = self.get_clock().now().to_msg()

        with self.lock:
            # Render RGB image
            self.renderer.update_scene(self.data, camera='d435_rgb')
            rgb_image = self.renderer.render()

            # Render depth image
            self.renderer.update_scene(self.data, camera='d435_depth')
            self.renderer.enable_depth_rendering()
            depth_image = self.renderer.render()
            self.renderer.disable_depth_rendering()

        # Publish RGB image
        try:
            rgb_msg = self.cv_bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
            rgb_msg.header.stamp = now
            rgb_msg.header.frame_id = 'd435_color_optical_frame'
            self.rgb_pub.publish(rgb_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish RGB image: {e}')

        # Publish depth image
        try:
            # Convert to 16-bit depth in mm
            depth_mm = (depth_image * 1000).astype(np.uint16)
            depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = 'd435_depth_optical_frame'
            self.depth_pub.publish(depth_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish depth image: {e}')

        # Publish camera info
        camera_info = CameraInfo()
        camera_info.header.stamp = now
        camera_info.header.frame_id = 'd435_color_optical_frame'
        camera_info.width = 640
        camera_info.height = 480
        # Approximate D435 intrinsics (fovy=42 degrees)
        fy = 480 / (2 * np.tan(np.radians(42) / 2))
        fx = fy  # Square pixels
        camera_info.k = [fx, 0.0, 320.0, 0.0, fy, 240.0, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, 320.0, 0.0, 0.0, fy, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(camera_info)

    def destroy_node(self):
        """Clean up resources."""
        if self.viewer is not None:
            self.viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MuJoCoBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
