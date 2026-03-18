#!/usr/bin/env python3
"""
Phone Teleoperation Server for TidyBot2 (WebXR).

Uses the same WebXR-based control scheme as the upstream tidybot2 project.
The phone runs a WebXR AR session providing full 6-DOF pose tracking.

Architecture:
    Phone (AR browser) <--Socket.IO--> Flask Server <--ROS2 topics--> Robot

Control Mapping (same as upstream):
    Touch left 90% of screen  → Arm control mode
    Touch right 10% of screen → Base control mode
    Vertical swipe (arm mode) → Gripper open/close

    Base: Phone tilt maps to robot base velocity (proportional speed control)
        - Tilt forward/back (pitch about X) → robot forward/backward
        - Tilt left/right (roll about Y)    → robot strafe left/right
        - Rotate flat about Z (yaw)         → robot rotation in place

    Arm: Tilt + translation control via IK (pinocchio)
        - Phone XY translation → EE position (scaled to 70%)
        - Phone tilt about X (fwd/back) → wrist angle velocity (in/out bend)
        - Phone tilt about Y (left/right) → wrist rotate velocity (CW/CCW)
        - IK solves all joints; tilt integrates into EE orientation target
        - Velocity limiting prevents jerky motion
        - Unreachable targets hold arm at last valid position

Usage:
    # Install web dependencies
    cd /home/locobot/collaborative-robotics-2026
    uv sync --extra web

    # Launch the robot (sim or real)
    ros2 launch tidybot_bringup sim.launch.py

    # Run the phone teleop server
    ros2 run tidybot_control phone_teleop_server

    # On iPhone: open http://<ROBOT_IP>:5000 in XRViewer or XR Browser
    # On Android: open in Chrome (WebXR support required)
"""

import math
import os
import queue
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit


# --- Constants ---
TWO_PI = 2.0 * math.pi
SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]
HOME_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# WX250s joint limits (radians)
JOINT_LIMITS = {
    'waist':        (-3.14, 3.14),
    'shoulder':     (-1.88, 1.99),
    'elbow':        (-2.15, 1.61),
    'forearm_roll': (-3.14, 3.14),
    'wrist_angle':  (-1.75, 2.15),
    'wrist_rotate': (-3.14, 3.14),
}
JOINT_NAMES = list(JOINT_LIMITS.keys())

# Control parameters
CONTROL_RATE = 20                # Hz
MAX_JOINT_VELOCITY = 0.5         # rad/s per joint (velocity limit)
MESSAGE_TIMEOUT_MS = 250         # Ignore stale WebXR messages

# Base velocity control parameters (tilt-based)
BASE_MAX_LINEAR_VEL = 0.5        # m/s cap
BASE_MAX_ANGULAR_VEL = 1.0       # rad/s cap
BASE_MAX_TILT_ANGLE = math.pi / 4  # 45 deg tilt = full speed
BASE_TILT_DEADZONE = 0.05        # radians (~3 deg) — ignore small tilts

# Arm control parameters
ARM_TRANSLATION_SCALE = 0.7      # 70% of phone displacement → EE displacement
ARM_MAX_WRIST_VEL = 1.5          # rad/s max EE orientation change from tilt
ARM_IK_POS_TOL = 0.05            # meters — reject IK solution if position error exceeds this

# WebXR coordinate conversion
DEVICE_CAMERA_OFFSET = np.array([0.0, 0.02, -0.04])


def convert_webxr_pose(pos, quat):
    """Convert WebXR pose to robot frame.
    WebXR: +x right, +y up, +z back
    Robot: +x forward, +y left, +z up
    """
    pos = np.array([-pos['z'], -pos['x'], pos['y']], dtype=np.float64)
    rot = R.from_quat([-quat['z'], -quat['x'], quat['y'], quat['w']])
    pos = pos + rot.apply(DEVICE_CAMERA_OFFSET)
    return pos, rot


# ---------------------------------------------------------------------------
# Arm IK Controller (pinocchio-based)
# ---------------------------------------------------------------------------

class ArmIKController:
    """Lightweight pinocchio-based FK/IK for WX250s arm teleop.

    Provides:
    - Forward kinematics: joint positions → EE pose
    - Inverse kinematics: target EE pose → joint positions (damped least-squares)
    - Velocity-limited joint interpolation
    """

    ARM_JOINTS = {
        'right': ['right_waist', 'right_shoulder', 'right_elbow',
                  'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate'],
        'left': ['left_waist', 'left_shoulder', 'left_elbow',
                 'left_forearm_roll', 'left_wrist_angle', 'left_wrist_rotate'],
    }
    EE_FRAMES = {'right': 'right_ee_arm_link', 'left': 'left_ee_arm_link'}

    def __init__(self, logger):
        self.logger = logger
        self.model = None
        self.data = None
        self.joint_ids = {}
        self.ee_frame_ids = {}
        self._load_model()

    def _load_model(self):
        """Load pinocchio model from URDF."""
        # Find URDF
        urdf_path = Path(__file__).parent.parent.parent / \
            'tidybot_description' / 'urdf' / 'tidybot_wx250s.urdf.xacro'
        if not urdf_path.exists():
            # Try installed share path
            import ament_index_python
            try:
                pkg_path = ament_index_python.get_package_share_directory('tidybot_description')
                urdf_path = Path(pkg_path) / 'urdf' / 'tidybot_wx250s.urdf.xacro'
            except Exception:
                pass

        if not urdf_path.exists():
            self.logger.error(f'URDF not found: {urdf_path}. Arm IK disabled.')
            return

        # Process xacro
        try:
            result = subprocess.run(
                ['xacro', str(urdf_path)], capture_output=True, text=True, check=True)
            urdf_string = result.stdout
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            self.logger.error(f'Xacro failed: {e}. Arm IK disabled.')
            return

        self.model = pin.buildModelFromXML(urdf_string)
        self.data = self.model.createData()

        # Map joint names to pinocchio IDs
        for arm in ['right', 'left']:
            for jname in self.ARM_JOINTS[arm]:
                if self.model.existJointName(jname):
                    self.joint_ids[jname] = self.model.getJointId(jname)

        # Map EE frame names
        for arm, frame_name in self.EE_FRAMES.items():
            if self.model.existFrame(frame_name):
                self.ee_frame_ids[arm] = self.model.getFrameId(frame_name)
            else:
                alt = f'{arm}_pinch_site'
                if self.model.existFrame(alt):
                    self.ee_frame_ids[arm] = self.model.getFrameId(alt)

        self.logger.info(f'Pinocchio model loaded: {self.model.nq} DOF')

    @property
    def available(self):
        return self.model is not None

    def _set_arm_in_q(self, q, arm, positions):
        """Set arm joint positions in full configuration vector."""
        for i, jname in enumerate(self.ARM_JOINTS[arm]):
            if jname in self.joint_ids:
                jid = self.joint_ids[jname]
                q[self.model.joints[jid].idx_q] = positions[i]

    def _get_arm_from_q(self, q, arm):
        """Extract arm joint positions from full configuration vector."""
        positions = np.zeros(6)
        for i, jname in enumerate(self.ARM_JOINTS[arm]):
            if jname in self.joint_ids:
                jid = self.joint_ids[jname]
                positions[i] = q[self.model.joints[jid].idx_q]
        return positions

    def forward_kinematics(self, arm, joint_positions):
        """Compute EE pose from joint positions.
        Returns (position[3], rotation_matrix[3x3]) in base_link frame.
        """
        if not self.available or arm not in self.ee_frame_ids:
            return None, None

        q = pin.neutral(self.model)
        self._set_arm_in_q(q, arm, joint_positions)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        ee_pose = self.data.oMf[self.ee_frame_ids[arm]]
        return ee_pose.translation.copy(), ee_pose.rotation.copy()

    def numerical_jacobian(self, q, arm, use_orientation=True, eps=1e-4):
        """Compute numerical Jacobian for the arm."""
        ee_frame_id = self.ee_frame_ids[arm]
        arm_idx_q = []
        for jname in self.ARM_JOINTS[arm]:
            if jname in self.joint_ids:
                jid = self.joint_ids[jname]
                arm_idx_q.append(self.model.joints[jid].idx_q)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pos0 = self.data.oMf[ee_frame_id].translation.copy()
        R0 = self.data.oMf[ee_frame_id].rotation.copy() if use_orientation else None

        rows = 6 if use_orientation else 3
        J = np.zeros((rows, len(arm_idx_q)))
        for i, idx_q in enumerate(arm_idx_q):
            q_plus = q.copy()
            q_plus[idx_q] += eps
            pin.forwardKinematics(self.model, self.data, q_plus)
            pin.updateFramePlacements(self.model, self.data)
            pos_plus = self.data.oMf[ee_frame_id].translation.copy()
            J[:3, i] = (pos_plus - pos0) / eps
            if use_orientation:
                R_plus = self.data.oMf[ee_frame_id].rotation.copy()
                J[3:, i] = pin.log3(R0.T @ R_plus) / eps

        return J

    def solve_ik(self, arm, target_pos, target_rot, seed,
                 other_arm=None, other_positions=None,
                 max_iter=50, dt=0.3, damping=1e-5,
                 pos_tol=0.01, ori_tol=0.1):
        """Solve IK via damped least-squares.
        Returns (success, joint_positions[6]).
        """
        if not self.available or arm not in self.ee_frame_ids:
            return False, seed

        q = pin.neutral(self.model)
        self._set_arm_in_q(q, arm, seed)
        if other_arm and other_positions is not None:
            self._set_arm_in_q(q, other_arm, other_positions)

        ee_frame_id = self.ee_frame_ids[arm]
        arm_idx_q = []
        for jname in self.ARM_JOINTS[arm]:
            if jname in self.joint_ids:
                jid = self.joint_ids[jname]
                arm_idx_q.append(self.model.joints[jid].idx_q)

        limits = list(JOINT_LIMITS.values())

        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[ee_frame_id]
            pos_err_vec = target_pos - current.translation
            pos_err = np.linalg.norm(pos_err_vec)
            ori_err_vec = pin.log3(target_rot.T @ current.rotation)
            ori_err = np.linalg.norm(ori_err_vec)

            if pos_err < pos_tol and ori_err < ori_tol:
                break

            error_vec = np.concatenate([pos_err_vec, -ori_err_vec])
            J = self.numerical_jacobian(q, arm, use_orientation=True)
            JJT = J @ J.T + damping * np.eye(6)

            try:
                v = J.T @ np.linalg.solve(JJT, error_vec)
            except np.linalg.LinAlgError:
                break

            for i, idx_q in enumerate(arm_idx_q):
                q[idx_q] = np.clip(q[idx_q] + dt * v[i], limits[i][0], limits[i][1])

        solution = self._get_arm_from_q(q, arm)
        for i in range(6):
            solution[i] = np.clip(solution[i], limits[i][0], limits[i][1])

        return True, solution


# ---------------------------------------------------------------------------
# Teleop Controller (WebXR base + cartesian arm)
# ---------------------------------------------------------------------------

def _tilt_to_vel(angle, deadzone, max_angle, max_vel):
    """Map a tilt angle to a velocity with deadzone and clamping."""
    if abs(angle) < deadzone:
        return 0.0
    frac = np.clip(angle / max_angle, -1.0, 1.0)
    return frac * max_vel


class TeleopController:
    """Processes WebXR messages and computes base velocity + arm targets.

    Supports dual-phone bimanual control: each phone is auto-assigned to an
    arm (right first, then left). Either phone can also do base control.
    """

    def __init__(self, ik_controller):
        self.ik = ik_controller

        # Device-to-arm mapping: {device_id: 'right'|'left'}
        self.device_arm_map = {}
        self.enabled_counts = {}

        # Per-arm teleop state
        self.arm_state = {
            'right': self._new_arm_state(),
            'left': self._new_arm_state(),
        }

        # Base control (shared — either phone can drive, latest wins)
        self.base_xr_ref_rot_inv = {}   # {device_id: rot_inv}
        self.base_cmd_vel = np.array([0.0, 0.0, 0.0])  # [vx, vy, wz]
        self.base_control_active = False
        self.base_control_device = None

    @staticmethod
    def _new_arm_state():
        return {
            'xr_ref_pos': None,
            'xr_ref_rot_inv': None,
            'ref_ee_pos': None,
            'ref_ee_rot': None,
            'current_target_rot': None,  # Evolving EE orientation (tilt-integrated)
            'target_joints': None,
            'control_active': False,
            'gripper_delta': 0.0,
        }

    def assign_device_to_arm(self, device_id, arm):
        """Assign a device to a specific arm, clearing any conflicts."""
        old_arm = self.device_arm_map.get(device_id)
        if old_arm:
            self.arm_state[old_arm] = self._new_arm_state()
        # Swap: if another device has the target arm, give it our old arm
        other_arm = 'left' if arm == 'right' else 'right'
        for did, a in list(self.device_arm_map.items()):
            if a == arm and did != device_id:
                self.device_arm_map[did] = other_arm
                self.arm_state[other_arm] = self._new_arm_state()
        self.device_arm_map[device_id] = arm

    def remove_device(self, device_id):
        """Clean up all state for a disconnected device."""
        arm = self.device_arm_map.pop(device_id, None)
        if arm:
            self.arm_state[arm] = self._new_arm_state()
        self.enabled_counts.pop(device_id, None)
        self.base_xr_ref_rot_inv.pop(device_id, None)
        if self.base_control_device == device_id:
            self.base_cmd_vel[:] = 0.0
            self.base_control_active = False
            self.base_control_device = None

    def _auto_assign_device(self, device_id):
        """Auto-assign device to first available arm (right, then left)."""
        if device_id in self.device_arm_map:
            return
        assigned_arms = set(self.device_arm_map.values())
        if 'right' not in assigned_arms:
            self.device_arm_map[device_id] = 'right'
        elif 'left' not in assigned_arms:
            self.device_arm_map[device_id] = 'left'
        # else: both arms taken, device won't get arm control

    def process_message(self, data, arm_positions):
        """Process a WebXR message from one device.

        Args:
            data: WebXR message dict with device_id, teleop_mode, position, orientation
            arm_positions: {'right': [6 floats], 'left': [6 floats]}
        """
        device_id = data.get('device_id', 'default')

        # Update enabled count (frames with active touch)
        if 'teleop_mode' in data:
            self.enabled_counts[device_id] = self.enabled_counts.get(device_id, 0) + 1
        else:
            self.enabled_counts[device_id] = 0

        # Auto-assign device to an arm after warmup frames
        if self.enabled_counts.get(device_id, 0) > 2:
            self._auto_assign_device(device_id)
        elif self.enabled_counts.get(device_id, 0) == 0:
            # Touch released — reset arm XR refs but keep device-to-arm mapping
            arm = self.device_arm_map.get(device_id)
            if arm:
                state = self.arm_state[arm]
                state['xr_ref_pos'] = None
                state['xr_ref_rot_inv'] = None
                state['ref_ee_pos'] = None
                state['ref_ee_rot'] = None
                state['current_target_rot'] = None
                state['control_active'] = False
                state['gripper_delta'] = 0.0
            # Reset base ref for this device
            self.base_xr_ref_rot_inv.pop(device_id, None)
            if self.base_control_device == device_id:
                self.base_cmd_vel[:] = 0.0
                self.base_control_active = False
                self.base_control_device = None

        # Process teleop commands
        if 'teleop_mode' not in data:
            return
        if 'position' not in data or 'orientation' not in data:
            return

        pos, rot = convert_webxr_pose(data['position'], data['orientation'])

        # --- BASE CONTROL (tilt-based velocity) ---
        if data['teleop_mode'] == 'base':
            if device_id not in self.base_xr_ref_rot_inv:
                self.base_xr_ref_rot_inv[device_id] = rot.inv()

            delta_rot = rot * self.base_xr_ref_rot_inv[device_id]
            rx, ry, rz = delta_rot.as_euler('xyz')

            vx = _tilt_to_vel(ry, BASE_TILT_DEADZONE,
                              BASE_MAX_TILT_ANGLE, BASE_MAX_LINEAR_VEL)
            vy = _tilt_to_vel(-rx, BASE_TILT_DEADZONE,
                              BASE_MAX_TILT_ANGLE, BASE_MAX_LINEAR_VEL)
            wz = _tilt_to_vel(rz, BASE_TILT_DEADZONE,
                              BASE_MAX_TILT_ANGLE, BASE_MAX_ANGULAR_VEL)

            self.base_cmd_vel = np.array([vx, vy, wz])
            self.base_control_active = True
            self.base_control_device = device_id

        # --- ARM CONTROL ---
        # Phone XY translation → EE position (scaled to 70%)
        # Phone tilt about X (fwd/back) → wrist angle (in/out bend), velocity-controlled
        # Phone tilt about Y (left/right) → wrist rotate (CW/CCW top-down), velocity-controlled
        # IK handles all joints for position; tilt integrates into EE orientation target
        elif data['teleop_mode'] == 'arm':
            arm = self.device_arm_map.get(device_id)
            if arm is None:
                return  # Device not assigned to any arm

            state = self.arm_state[arm]
            current_arm_positions = arm_positions[arm]

            if state['xr_ref_pos'] is None:
                state['xr_ref_pos'] = pos.copy()
                state['xr_ref_rot_inv'] = rot.inv()

                if self.ik.available:
                    ee_pos, ee_rot = self.ik.forward_kinematics(
                        arm, np.array(current_arm_positions))
                    if ee_pos is not None:
                        state['ref_ee_pos'] = ee_pos.copy()
                        state['ref_ee_rot'] = ee_rot.copy()
                        state['current_target_rot'] = ee_rot.copy()

            if state['ref_ee_pos'] is not None and self.ik.available:
                # Position: phone XY translation → EE XY (keep Z at reference)
                delta_pos = pos - state['xr_ref_pos']
                target_ee_pos = state['ref_ee_pos'].copy()
                target_ee_pos[0] += delta_pos[0] * ARM_TRANSLATION_SCALE
                target_ee_pos[1] += delta_pos[1] * ARM_TRANSLATION_SCALE

                # Orientation: phone tilt → wrist velocity (integrated each frame)
                phone_delta_rot = rot * state['xr_ref_rot_inv']
                rx, ry, rz = phone_delta_rot.as_euler('xyz')
                dt = 1.0 / CONTROL_RATE

                # ry (forward/back tilt) → wrist angle change (in/out bend)
                omega_pitch = _tilt_to_vel(ry, BASE_TILT_DEADZONE,
                                           BASE_MAX_TILT_ANGLE, ARM_MAX_WRIST_VEL)
                # rx (left/right tilt) → wrist rotate change (CW/CCW top-down)
                omega_yaw = _tilt_to_vel(-rx, BASE_TILT_DEADZONE,
                                         BASE_MAX_TILT_ANGLE, ARM_MAX_WRIST_VEL)

                # Save previous orientation in case IK fails (revert to stay reachable)
                prev_target_rot = state['current_target_rot'].copy()

                # Integrate: apply small rotation in robot frame
                d_rot = R.from_rotvec([0, omega_pitch * dt, omega_yaw * dt])
                state['current_target_rot'] = d_rot.as_matrix() @ state['current_target_rot']

                # Solve IK for position + tilt-integrated orientation
                success, joints = self.ik.solve_ik(
                    arm,
                    target_ee_pos, state['current_target_rot'],
                    seed=np.array(current_arm_positions),
                    max_iter=30, dt=0.3, damping=1e-5)

                if success:
                    # Verify IK solution actually reaches the target position
                    solved_pos, _ = self.ik.forward_kinematics(arm, joints)
                    if (solved_pos is not None and
                            np.linalg.norm(solved_pos - target_ee_pos) < ARM_IK_POS_TOL):
                        state['target_joints'] = joints
                        state['control_active'] = True
                    else:
                        # Unreachable — revert orientation so it doesn't drift
                        state['current_target_rot'] = prev_target_rot
                else:
                    state['current_target_rot'] = prev_target_rot

            state['gripper_delta'] = data.get('gripper_delta', 0.0)


class PhoneTeleopNode(Node):
    """ROS2 node that bridges WebXR phone commands to robot topics."""

    def __init__(self):
        super().__init__('phone_teleop_server')

        self.declare_parameter('port', 5000)
        self.declare_parameter('host', '0.0.0.0')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.right_arm_pub = self.create_publisher(Float64MultiArray, '/right_arm/joint_cmd', 10)
        self.left_arm_pub = self.create_publisher(Float64MultiArray, '/left_arm/joint_cmd', 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, '/right_gripper/cmd', 10)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, '/left_gripper/cmd', 10)
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # Subscribers
        self.right_arm_positions = list(SLEEP_POSE)
        self.left_arm_positions = list(SLEEP_POSE)
        self.create_subscription(JointState, '/right_arm/joint_states', self._right_js_cb, 10)
        self.create_subscription(JointState, '/left_arm/joint_states', self._left_js_cb, 10)

        # IK controller
        self.ik = ArmIKController(self.get_logger())

        # Teleop controller
        self.teleop = TeleopController(self.ik)

        # Per-device message queues from Socket.IO
        self.msg_queues = {}            # {device_id: Queue}
        self.msg_queues_lock = threading.Lock()
        self.sid_device_map = {}        # SocketIO sid → device_id

        # Shared state
        self.lock = threading.Lock()
        self.gripper_state = {'right': 0.0, 'left': 0.0}

        # Velocity-limited arm state
        self.current_cmd_positions = {'right': None, 'left': None}

        # Control timer
        self.control_timer = self.create_timer(1.0 / CONTROL_RATE, self._control_loop)

        self.get_logger().info('PhoneTeleopNode initialized (dual-phone bimanual teleop)')

    def _right_js_cb(self, msg):
        if len(msg.position) >= 6:
            self.right_arm_positions = list(msg.position[:6])

    def _left_js_cb(self, msg):
        if len(msg.position) >= 6:
            self.left_arm_positions = list(msg.position[:6])

    def _control_loop(self):
        """Process messages and publish velocity-limited commands for both arms."""
        dt = 1.0 / CONTROL_RATE

        arm_positions = {
            'right': list(self.right_arm_positions),
            'left': list(self.left_arm_positions),
        }

        # Drain all per-device queues → latest non-stale message per device
        latest_per_device = {}
        with self.msg_queues_lock:
            device_ids = list(self.msg_queues.keys())
        for device_id in device_ids:
            with self.msg_queues_lock:
                q = self.msg_queues.get(device_id)
            if q is None:
                continue
            latest = None
            while not q.empty():
                try:
                    data = q.get_nowait()
                    if 'timestamp' in data:
                        age_ms = time.time() * 1000 - data['timestamp']
                        if age_ms > MESSAGE_TIMEOUT_MS:
                            continue
                    latest = data
                except queue.Empty:
                    break
            if latest is not None:
                latest_per_device[device_id] = latest

        # Process each device's latest message
        for device_id, data in latest_per_device.items():
            self.teleop.process_message(data, arm_positions)

        # --- Base: publish cmd_vel ---
        twist = Twist()
        if self.teleop.base_control_active:
            twist.linear.x = float(self.teleop.base_cmd_vel[0])
            twist.linear.y = float(self.teleop.base_cmd_vel[1])
            twist.angular.z = float(self.teleop.base_cmd_vel[2])
        self.cmd_vel_pub.publish(twist)

        # --- Both arms: velocity-limited interpolation toward IK targets ---
        limits = list(JOINT_LIMITS.values())
        for arm in ['right', 'left']:
            state = self.teleop.arm_state[arm]
            if state['control_active'] and state['target_joints'] is not None:
                target = state['target_joints']

                if self.current_cmd_positions[arm] is None:
                    self.current_cmd_positions[arm] = np.array(arm_positions[arm])

                cmd = self.current_cmd_positions[arm].copy()
                for i in range(6):
                    diff = target[i] - cmd[i]
                    max_step = MAX_JOINT_VELOCITY * dt
                    if abs(diff) > max_step:
                        diff = math.copysign(max_step, diff)
                    cmd[i] += diff
                    lo, hi = limits[i]
                    cmd[i] = max(lo, min(hi, cmd[i]))
                self.current_cmd_positions[arm] = cmd

                msg = Float64MultiArray()
                msg.data = cmd.tolist()
                if arm == 'right':
                    self.right_arm_pub.publish(msg)
                else:
                    self.left_arm_pub.publish(msg)

                # Gripper from vertical swipe
                gripper_delta = state['gripper_delta']
                if abs(gripper_delta) > 0.3:
                    new_state = 1.0 if gripper_delta < -0.3 else 0.0
                    with self.lock:
                        if self.gripper_state[arm] != new_state:
                            self.gripper_state[arm] = new_state
                            grip_msg = Float64MultiArray()
                            grip_msg.data = [new_state]
                            if arm == 'right':
                                self.right_gripper_pub.publish(grip_msg)
                            else:
                                self.left_gripper_pub.publish(grip_msg)
            else:
                self.current_cmd_positions[arm] = None

    def assign_device_to_arm(self, device_id, arm):
        with self.lock:
            self.teleop.assign_device_to_arm(device_id, arm)
        self.get_logger().info(f'Device {device_id[:8]}... -> {arm} arm')

    def toggle_gripper(self, side):
        with self.lock:
            current = self.gripper_state[side]
            new_state = 0.0 if current > 0.5 else 1.0
            self.gripper_state[side] = new_state
        msg = Float64MultiArray()
        msg.data = [new_state]
        if side == 'right':
            self.right_gripper_pub.publish(msg)
        else:
            self.left_gripper_pub.publish(msg)
        self.get_logger().info(f'{side} gripper -> {"CLOSED" if new_state > 0.5 else "OPEN"}')

    def send_arm_to_pose(self, arm, pose):
        msg = Float64MultiArray()
        msg.data = pose
        if arm == 'right':
            self.right_arm_pub.publish(msg)
        else:
            self.left_arm_pub.publish(msg)
        self.get_logger().info(f'Sent {arm} arm to preset pose')

    def emergency_stop(self):
        for arm in ['right', 'left']:
            self.teleop.arm_state[arm] = self.teleop._new_arm_state()
        self.teleop.base_cmd_vel[:] = 0.0
        self.teleop.base_control_active = False
        self.teleop.base_control_device = None
        self.teleop.device_arm_map.clear()
        self.teleop.base_xr_ref_rot_inv.clear()
        self.teleop.enabled_counts.clear()
        self.current_cmd_positions = {'right': None, 'left': None}
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('EMERGENCY STOP')


# ---------------------------------------------------------------------------
# Flask + Socket.IO app
# ---------------------------------------------------------------------------

def create_flask_app(ros_node: PhoneTeleopNode):
    app = Flask(__name__)
    app.config['SECRET_KEY'] = 'tidybot2-teleop'
    socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

    @app.route('/')
    def index():
        return render_template_string(WEBXR_HTML)

    @app.route('/static/js/<path:filename>')
    def serve_static_js(filename):
        if filename == 'webxr-button.js':
            return WEBXR_BUTTON_JS, 200, {'Content-Type': 'application/javascript'}
        return '', 404

    @socketio.on('connect')
    def handle_connect():
        ros_node.get_logger().info('Phone connected')

    @socketio.on('disconnect')
    def handle_disconnect():
        from flask import request
        sid = request.sid
        device_id = ros_node.sid_device_map.pop(sid, None)
        if device_id:
            ros_node.teleop.remove_device(device_id)
            with ros_node.msg_queues_lock:
                ros_node.msg_queues.pop(device_id, None)
            ros_node.get_logger().info(f'Phone disconnected (device {device_id[:8]}...)')
        else:
            ros_node.get_logger().info('Phone disconnected (unknown device)')

    @socketio.on('message')
    def handle_message(data):
        if 'timestamp' in data:
            emit('echo', data['timestamp'])
        if 'state_update' in data:
            ros_node.get_logger().info(f'State: {data["state_update"]}')
            return
        device_id = data.get('device_id', 'default')
        # Track SocketIO sid → device_id for disconnect cleanup
        from flask import request
        ros_node.sid_device_map[request.sid] = device_id
        # Route to per-device queue
        with ros_node.msg_queues_lock:
            if device_id not in ros_node.msg_queues:
                ros_node.msg_queues[device_id] = queue.Queue(maxsize=20)
        try:
            ros_node.msg_queues[device_id].put_nowait(data)
        except queue.Full:
            pass

    @socketio.on('set_arm')
    def handle_set_arm(data):
        device_id = data.get('device_id', 'default')
        arm = data.get('arm', 'right')
        ros_node.assign_device_to_arm(device_id, arm)
        # Broadcast assignment to all clients
        socketio.emit('arm_assignment', {
            'device_id': device_id,
            'arm': arm,
            'all_assignments': dict(ros_node.teleop.device_arm_map),
        })

    @socketio.on('request_assignment')
    def handle_request_assignment(data):
        device_id = data.get('device_id', 'default')
        arm = ros_node.teleop.device_arm_map.get(device_id)
        emit('arm_assignment', {
            'device_id': device_id,
            'arm': arm,
            'all_assignments': dict(ros_node.teleop.device_arm_map),
        })

    @socketio.on('toggle_gripper')
    def handle_toggle_gripper(data):
        device_id = data.get('device_id', 'default')
        default_arm = ros_node.teleop.device_arm_map.get(device_id, 'right')
        ros_node.toggle_gripper(data.get('side', default_arm))

    @socketio.on('preset_pose')
    def handle_preset_pose(data):
        device_id = data.get('device_id', 'default')
        default_arm = ros_node.teleop.device_arm_map.get(device_id, 'right')
        arm = data.get('arm', default_arm)
        pose_name = data.get('pose', 'sleep')
        pose = SLEEP_POSE if pose_name == 'sleep' else HOME_POSE
        ros_node.send_arm_to_pose(arm, pose)

    @socketio.on('emergency_stop')
    def handle_emergency_stop():
        ros_node.emergency_stop()

    return app, socketio


# ---------------------------------------------------------------------------
# WebXR Button JS (Google Apache 2.0 license)
# ---------------------------------------------------------------------------

WEBXR_BUTTON_JS = r"""
const _LOGO_SCALE = 0.8;
let _WEBXR_UI_CSS_INJECTED = {};

const generateInnerHTML = (cssPrefix, height) => {
  const logoHeight = height * _LOGO_SCALE;
  const svgString = generateXRIconString(cssPrefix, logoHeight) + generateNoXRIconString(cssPrefix, logoHeight);
  return `<button class="${cssPrefix}-button">
    <div class="${cssPrefix}-title"></div>
    <div class="${cssPrefix}-logo">${svgString}</div>
  </button>`;
};

const injectCSS = (cssText) => {
  const style = document.createElement('style');
  style.innerHTML = cssText;
  let head = document.getElementsByTagName('head')[0];
  head.insertBefore(style, head.firstChild);
};

const createDefaultView = (options) => {
  const fontSize = options.height / 3;
  if (options.injectCSS) {
    if (!_WEBXR_UI_CSS_INJECTED[options.cssprefix]) {
      injectCSS(generateCSS(options, fontSize));
      _WEBXR_UI_CSS_INJECTED[options.cssprefix] = true;
    }
  }
  const el = document.createElement('div');
  el.innerHTML = generateInnerHTML(options.cssprefix, fontSize);
  return el.firstChild;
};

const generateXRIconString = (cssPrefix, height) => {
  let aspect = 28 / 18;
  return `<svg class="${cssPrefix}-svg" version="1.1" x="0px" y="0px"
    width="${aspect * height}px" height="${height}px" viewBox="0 0 28 18" xml:space="preserve">
    <path d="M26.8,1.1C26.1,0.4,25.1,0,24.2,0H3.4c-1,0-1.7,0.4-2.4,1.1C0.3,1.7,0,2.7,0,3.6v10.7
    c0,1,0.3,1.9,0.9,2.6C1.6,17.6,2.4,18,3.4,18h5c0.7,0,1.3-0.2,1.8-0.5c0.6-0.3,1-0.8,1.3-1.4l
    1.5-2.6C13.2,13.1,13,13,14,13v0h-0.2h0c0.3,0,0.7,0.1,0.8,0.5l1.4,2.6c0.3,0.6,0.8,1.1,1.3,
    1.4c0.6,0.3,1.2,0.5,1.8,0.5h5c1,0,2-0.4,2.7-1.1c0.7-0.7,1.2-1.6,1.2-2.6V3.6C28,2.7,27.5,
    1.7,26.8,1.1z M7.4,11.8c-1.6,0-2.8-1.3-2.8-2.8c0-1.6,1.3-2.8,2.8-2.8c1.6,0,2.8,1.3,2.8,2.8
    C10.2,10.5,8.9,11.8,7.4,11.8z M20.1,11.8c-1.6,0-2.8-1.3-2.8-2.8c0-1.6,1.3-2.8,2.8-2.8C21.7,
    6.2,23,7.4,23,9C23,10.5,21.7,11.8,20.1,11.8z"/>
  </svg>`;
};

const generateNoXRIconString = (cssPrefix, height) => {
  let aspect = 28 / 18;
  return `<svg class="${cssPrefix}-svg-error" x="0px" y="0px"
    width="${aspect * height}px" height="${aspect * height}px" viewBox="0 0 28 28" xml:space="preserve">
    <path d="M17.6,13.4c0-0.2-0.1-0.4-0.1-0.6c0-1.6,1.3-2.8,2.8-2.8s2.8,1.3,2.8,2.8s-1.3,2.8-2.8,2.8
    c-0.2,0-0.4,0-0.6-0.1l5.9,5.9c0.5-0.2,0.9-0.4,1.3-0.8c0.7-0.7,1.1-1.6,1.1-2.5V7.4c0-1-0.4-1.9-1.1-2.5
    c-0.7-0.7-1.6-1-2.5-1H8.1L17.6,13.4z"/>
    <path d="M10.1,14.2c-0.5,0.9-1.4,1.4-2.4,1.4c-1.6,0-2.8-1.3-2.8-2.8c0-1.1,0.6-2,1.4-2.5L0.9,5.1
    C0.3,5.7,0,6.6,0,7.5v10.7c0,1,0.4,1.8,1.1,2.5c0.7,0.7,1.6,1,2.5,1h5c0.7,0,1.3-0.1,1.8-0.5
    c0.6-0.3,1-0.8,1.3-1.4l1.3-2.6L10.1,14.2z"/>
    <path d="M25.5,27.5l-25-25C-0.1,2-0.1,1,0.5,0.4l0,0C1,-0.1,2,-0.1,2.6,0.4l25,25c0.6,0.6,0.6,1.5,0,2.1
    l0,0C27,28.1,26,28.1,25.5,27.5z"/>
  </svg>`;
};

const generateCSS = (options, fontSize = 18) => {
  const height = options.height;
  const borderColor = options.background ? options.background : options.color;
  const cssPrefix = options.cssprefix;
  let borderRadius = options.corners == 'round' ? options.height / 2 :
    options.corners == 'square' ? 2 : options.corners;

  return `
    button.${cssPrefix}-button {
      font-family: 'Karla', sans-serif;
      border: ${borderColor} 2px solid;
      border-radius: ${borderRadius}px;
      box-sizing: border-box;
      background: ${options.background ? options.background : 'none'};
      height: ${height}px;
      min-width: ${fontSize * 9.6}px;
      display: inline-block;
      position: relative;
      cursor: pointer;
      transition: border 0.5s;
    }
    button.${cssPrefix}-button:focus { outline: none; }
    .${cssPrefix}-logo {
      width: ${height}px; height: ${height}px;
      position: absolute; top: 0; left: 0;
      width: ${height - 4}px; height: ${height - 4}px;
    }
    .${cssPrefix}-svg {
      fill: ${options.color};
      margin-top: ${(height - fontSize * _LOGO_SCALE) / 2 - 2}px;
      margin-left: ${height / 3}px;
    }
    .${cssPrefix}-svg-error {
      fill: ${options.color}; display: none;
      margin-top: ${(height - 28 / 18 * fontSize * _LOGO_SCALE) / 2 - 2}px;
      margin-left: ${height / 3}px;
    }
    .${cssPrefix}-title {
      color: ${options.color};
      position: relative;
      font-size: ${fontSize}px;
      padding-left: ${height * 1.05}px;
      padding-right: ${(borderRadius - 10 < 5) ? height / 3 : borderRadius - 10}px;
      transition: color 0.5s;
    }
    button.${cssPrefix}-button[disabled=true] { opacity: ${options.disabledOpacity}; }
    button.${cssPrefix}-button[disabled=true] > .${cssPrefix}-logo > .${cssPrefix}-svg { display: none; }
    button.${cssPrefix}-button[disabled=true] > .${cssPrefix}-logo > .${cssPrefix}-svg-error { display: initial; }
  `;
};

const ifChild = (el, cssPrefix, suffix, fn) => {
  const c = el.querySelector('.' + cssPrefix + '-' + suffix);
  c && fn(c);
};

export class WebXRButton {
  constructor(options) {
    options = options || {};
    options.color = options.color || 'rgb(80,168,252)';
    options.background = options.background || false;
    options.disabledOpacity = options.disabledOpacity || 0.5;
    options.height = options.height || 55;
    options.corners = options.corners || 'square';
    options.cssprefix = options.cssprefix || 'webvr-ui';
    options.textEnterXRTitle = options.textEnterXRTitle || 'ENTER VR';
    options.textXRNotFoundTitle = options.textXRNotFoundTitle || 'VR NOT FOUND';
    options.textPrepareExitXRTitle = options.textPrepareExitXRTitle || 'PREPARE EXIT VR';
    options.textExitXRTitle = options.textExitXRTitle || 'EXIT VR';
    options.onRequestSession = options.onRequestSession || function() {};
    options.onPrepareEndSession = options.onPrepareEndSession || function() {};
    options.onEndSession = options.onEndSession || function() {};
    options.injectCSS = options.injectCSS !== false;
    this.options = options;
    this._enabled = false;
    this.session = null;
    this.isPreparingToExit = false;
    this.domElement = options.domElement || createDefaultView(options);
    this.__defaultDisplayStyle = this.domElement.style.display || 'initial';
    this.domElement.addEventListener('click', () => this.__onXRButtonClick());
    this.__forceDisabled = false;
    this.__setDisabledAttribute(true);
    this.setTitle(this.options.textXRNotFoundTitle);
  }
  set enabled(enabled) { this._enabled = enabled; this.__updateButtonState(); return this; }
  get enabled() { return this._enabled; }
  setSession(session) { this.session = session; this.__updateButtonState(); return this; }
  setTitle(text) {
    this.domElement.title = text;
    ifChild(this.domElement, this.options.cssprefix, 'title', (title) => {
      if (!text) { title.style.display = 'none'; }
      else { title.innerText = text; title.style.display = 'initial'; }
    });
    return this;
  }
  setTooltip(tooltip) { this.domElement.title = tooltip; return this; }
  show() { this.domElement.style.display = this.__defaultDisplayStyle; return this; }
  hide() { this.domElement.style.display = 'none'; return this; }
  enable() { this.__setDisabledAttribute(false); this.__forceDisabled = false; return this; }
  disable() { this.__setDisabledAttribute(true); this.__forceDisabled = true; return this; }
  remove() { if (this.domElement.parentElement) this.domElement.parentElement.removeChild(this.domElement); }
  __setDisabledAttribute(disabled) {
    if (disabled || this.__forceDisabled) this.domElement.setAttribute('disabled', 'true');
    else this.domElement.removeAttribute('disabled');
  }
  __onXRButtonClick() {
    if (this.session && !this.isPreparingToExit) {
      this.options.onPrepareEndSession();
      this.isPreparingToExit = true;
      this.__updateButtonState();
    } else if (this.session && this.isPreparingToExit) {
      this.isPreparingToExit = false;
      this.options.onEndSession(this.session);
    } else if (this._enabled) {
      let p = this.options.onRequestSession();
      if (p) p.catch((err) => {
        this.setTooltip(`XRSession creation failed: ${err.message}`);
        this.__setDisabledAttribute(true);
        this.domElement.setAttribute('error', 'true');
        setTimeout(() => { this.__setDisabledAttribute(false); this.domElement.setAttribute('error', 'false'); }, 1000);
      });
    }
  }
  __updateButtonState() {
    if (this.session && !this.isPreparingToExit) { this.setTitle(this.options.textPrepareExitXRTitle); this.__setDisabledAttribute(false); }
    else if (this.session && this.isPreparingToExit) { this.setTitle(this.options.textExitXRTitle); this.__setDisabledAttribute(false); }
    else if (this._enabled) { this.setTitle(this.options.textEnterXRTitle); this.__setDisabledAttribute(false); }
    else { this.setTitle(this.options.textXRNotFoundTitle); this.__setDisabledAttribute(true); }
  }
}
"""


# ---------------------------------------------------------------------------
# WebXR HTML template
# ---------------------------------------------------------------------------

WEBXR_HTML = r"""<!doctype html>
<html>
<head>
  <meta charset='utf-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1, user-scalable=no'>
  <title>TidyBot2 Phone Teleop</title>
  <style>
    body {
      background-color: #F0F0F0;
      font: 1rem/1.4 -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Oxygen,
        Ubuntu, Cantarell, Fira Sans, Droid Sans, Helvetica Neue, sans-serif;
    }
    header { padding: 0.5em; background-color: rgba(255,255,255,0.90); }
    #info { font-size: 1.25em; background-color: rgba(240,240,240,0.5); }
    #arm-controls { padding: 0.4em; background-color: rgba(255,255,255,0.85); }
    #arm-controls { display: flex; flex-wrap: nowrap; align-items: center; gap: 3px; }
    #arm-controls button {
      padding: 5px 8px; border-radius: 5px;
      border: 2px solid #333; background: #fff; font-size: 11px;
      font-weight: 600; cursor: pointer; white-space: nowrap;
    }
    #arm-controls button.active { background: #4a90d9; color: #fff; border-color: #4a90d9; }
    #arm-controls .grip-btn { background: #27ae60; color: #fff; border-color: #27ae60; }
    #arm-controls .grip-btn.closed { background: #e74c3c; border-color: #e74c3c; }
    #arm-controls .preset-btn { background: #95a5a6; color: #fff; border-color: #7f8c8d; }
    #arm-controls .stop-btn { background: #c0392b; color: #fff; border-color: #a93226; font-size: 11px; }
    canvas {
      position: absolute; z-index: 0;
      width: 100%; height: 100%;
      left: 0; top: 0; right: 0; bottom: 0;
      margin: 0; touch-action: none;
    }
  </style>
</head>
<body>
  <div id="overlay">
    <header></header>
    <div id="arm-controls">
      <span id="arm-label" style="font-weight:700; font-size:11px; color:#e67e22; white-space:nowrap;">Not assigned</span>
      <button id="btn-right" onclick="selectArm('right')">Right</button>
      <button id="btn-left" onclick="selectArm('left')">Left</button>
      <button class="grip-btn" id="grip-btn" onclick="toggleGripper()">Gripper: OPEN</button>
      <button class="preset-btn" onclick="presetPose('home')">Home</button>
      <button class="preset-btn" onclick="presetPose('sleep')">Sleep</button>
      <button class="stop-btn" onclick="emergencyStop()">STOP</button>
    </div>
    <p><span id="info"></span></p>
  </div>
  <script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
  <script type="text/javascript">
    const deviceId = Math.random().toString(36).substring(2, 15);
    class RTTStats {
      constructor(n) { this.n = n; this.i = 0; this.buf = new Array(n).fill(0); }
      calculate(rtt) {
        this.buf[this.i] = rtt; this.i = (this.i + 1) % this.n;
        const min = Math.min(...this.buf), avg = this.buf.reduce((a,c)=>a+c,0)/this.n;
        const max = Math.max(...this.buf);
        return `RTT: ${min.toFixed(1)}/${avg.toFixed(1)}/${max.toFixed(1)} ms`;
      }
    }
    const rttStats = new RTTStats(100);
    const socket = io();
    socket.on('echo', (ts) => { document.getElementById('info').innerText = rttStats.calculate(Date.now()-ts); });

    // Per-phone arm assignment (set by server)
    let myArm = null;

    socket.on('connect', () => {
      socket.emit('request_assignment', {device_id: deviceId});
    });

    socket.on('arm_assignment', (data) => {
      const assignments = data.all_assignments || {};
      if (assignments[deviceId] !== undefined) {
        myArm = assignments[deviceId];
      } else if (data.device_id === deviceId) {
        myArm = data.arm;
      }
      updateArmUI();
    });

    function updateArmUI() {
      const label = document.getElementById('arm-label');
      if (myArm) {
        label.textContent = 'My arm: ' + myArm.toUpperCase();
        label.style.color = myArm === 'right' ? '#4a90d9' : '#27ae60';
      } else {
        label.textContent = 'Not assigned';
        label.style.color = '#e67e22';
      }
      document.getElementById('btn-right').classList.toggle('active', myArm === 'right');
      document.getElementById('btn-left').classList.toggle('active', myArm === 'left');
    }

    function selectArm(arm) {
      socket.emit('set_arm', {arm, device_id: deviceId});
    }
    let gripperOpen = true;
    function toggleGripper() {
      socket.emit('toggle_gripper', {side: myArm, device_id: deviceId}); gripperOpen = !gripperOpen;
      const btn = document.getElementById('grip-btn');
      btn.textContent = gripperOpen ? 'Gripper: OPEN' : 'Gripper: CLOSED';
      btn.classList.toggle('closed', !gripperOpen);
    }
    function presetPose(name) { socket.emit('preset_pose', {arm: myArm, pose: name, device_id: deviceId}); }
    function emergencyStop() { socket.emit('emergency_stop'); }
  </script>
  <script type="module">
    import { WebXRButton } from '/static/js/webxr-button.js';
    let xrButton = null, xrRefSpace = null, gl = null;

    function initXR() {
      xrButton = new WebXRButton({
        onRequestSession, onPrepareEndSession, onEndSession,
        textEnterXRTitle: 'Start episode', textXRNotFoundTitle: 'AR NOT FOUND',
        textPrepareExitXRTitle: 'End episode', textExitXRTitle: 'Reset env',
      });
      document.querySelector('header').appendChild(xrButton.domElement);
      if (navigator.xr) navigator.xr.isSessionSupported('immersive-ar').then(s => { xrButton.enabled = s; });
    }

    function onRequestSession() {
      return navigator.xr.requestSession('immersive-ar', {
        optionalFeatures: ['dom-overlay'], domOverlay: {root: document.getElementById('overlay')},
      }).then(session => { xrButton.setSession(session); session.isImmersive = true; onSessionStarted(session); });
    }
    function onSessionStarted(session) {
      session.addEventListener('end', onSessionEnded);
      const canvas = document.createElement('canvas');
      gl = canvas.getContext('webgl', {xrCompatible: true});
      addCanvasListeners(gl.canvas);
      session.updateRenderState({baseLayer: new XRWebGLLayer(session, gl)});
      session.requestReferenceSpace('local').then(ref => { xrRefSpace = ref; session.requestAnimationFrame(onXRFrame); });
      socket.send({timestamp: Date.now(), state_update: 'episode_started'});
    }
    function onPrepareEndSession() { socket.send({timestamp: Date.now(), state_update: 'episode_ended'}); }
    function onEndSession(session) { session.end(); socket.send({timestamp: Date.now(), state_update: 'reset_env'}); }
    function onSessionEnded() { xrButton.setSession(null); gl = null; }

    let touchId, touchStartY, touchDeltaY, teleopMode;
    function addCanvasListeners(canvas) {
      function handleTouch(t) {
        touchDeltaY = (touchStartY - t.clientY) / (0.2 * window.innerHeight);
        touchDeltaY = Math.min(1, Math.max(-1, touchDeltaY));
      }
      canvas.addEventListener('touchstart', e => {
        if (touchId === undefined) {
          const t = e.changedTouches[0]; touchId = t.identifier; touchStartY = t.clientY;
          teleopMode = t.clientX < 0.9 * window.innerWidth ? 'arm' : 'base';
          handleTouch(t);
        }
      });
      canvas.addEventListener('touchmove', e => { for (const t of e.changedTouches) if (touchId === t.identifier) handleTouch(t); });
      function clearTouch(e) { for (const t of e.changedTouches) if (touchId === t.identifier) touchId = undefined; }
      canvas.addEventListener('touchend', clearTouch);
      canvas.addEventListener('touchcancel', clearTouch);
    }

    function onXRFrame(t, frame) {
      frame.session.requestAnimationFrame(onXRFrame);
      const r = (touchId !== undefined && teleopMode === 'base') ? 0.25 : 0;
      const b = (touchId !== undefined && teleopMode === 'arm')
        ? (Math.abs(touchDeltaY) === 1 ? 1 : 0.25 + 0.25 * touchDeltaY) : 0;
      gl.clearColor(r, 0, b, 0.5); gl.clear(gl.COLOR_BUFFER_BIT);

      const data = {timestamp: Date.now(), device_id: deviceId};
      if (touchId !== undefined) {
        const pose = frame.getViewerPose(xrRefSpace);
        if (pose) {
          data.teleop_mode = teleopMode;
          data.position = {x: pose.transform.inverse.position.x, y: pose.transform.inverse.position.y, z: pose.transform.inverse.position.z};
          data.orientation = {x: pose.transform.inverse.orientation.x, y: pose.transform.inverse.orientation.y, z: pose.transform.inverse.orientation.z, w: pose.transform.inverse.orientation.w};
        }
        if (teleopMode === 'arm') data.gripper_delta = touchDeltaY;
      }
      socket.send(data);
    }

    initXR();
  </script>
</body>
</html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = PhoneTeleopNode()

    port = node.get_parameter('port').value
    host = node.get_parameter('host').value

    app, socketio = create_flask_app(node)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    node.get_logger().info(f'Starting phone teleop server on http://{host}:{port}')
    node.get_logger().info('Open this URL on your phone (same network)')
    node.get_logger().info('Requires AR browser: XRViewer (iOS) or Chrome (Android with ARCore)')

    try:
        socketio.run(app, host=host, port=port, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
