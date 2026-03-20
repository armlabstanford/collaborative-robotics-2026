"""Arm teleop: phone-to-EE IK-based control for WX250s arms.

Handles pinocchio model loading, FK, numerical Jacobian, damped
least-squares IK with nullspace optimization, and per-arm teleop state.
"""

import math
import subprocess
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R


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

# Arm control parameters
ARM_IK_POS_TOL = 0.05            # meters — reject IK solution if position error exceeds this
ARM_IK_MAX_STEP = math.radians(45)  # max joint angle change per IK iteration
ARM_IK_NULLSPACE_GAIN = 0.15    # gentle bias toward rest pose (was 0.5 — too strong)
# Rest pose for nullspace (shoulder-neutral so it doesn't fight either direction)
REST_POSE = np.array([0.0, 0.05, 0.5, 0.0, 0.0, 0.0])


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
        """Solve IK via damped least-squares with nullspace optimization.

        Nullspace projection biases redundant DOFs toward REST_POSE to
        avoid singularities and keep the arm in a comfortable configuration.
        Per-iteration joint step is clamped to ARM_IK_MAX_STEP.

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

        n_joints = len(arm_idx_q)
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
                Jinv = J.T @ np.linalg.solve(JJT, np.eye(6))
            except np.linalg.LinAlgError:
                break

            # Primary task: track target pose
            v = Jinv @ error_vec

            # Nullspace projection: bias toward rest pose
            nullspace = np.eye(n_joints) - Jinv @ J
            q_arm = np.array([q[idx] for idx in arm_idx_q])
            v += ARM_IK_NULLSPACE_GAIN * nullspace @ (REST_POSE - q_arm)

            # Clamp max step per iteration
            max_abs = np.max(np.abs(v))
            if max_abs > ARM_IK_MAX_STEP:
                v *= ARM_IK_MAX_STEP / max_abs

            for i, idx_q in enumerate(arm_idx_q):
                q[idx_q] = np.clip(q[idx_q] + dt * v[i], limits[i][0], limits[i][1])

        solution = self._get_arm_from_q(q, arm)
        for i in range(6):
            solution[i] = np.clip(solution[i], limits[i][0], limits[i][1])

        return True, solution


# ---------------------------------------------------------------------------
# Per-arm teleop state
# ---------------------------------------------------------------------------

class ArmTeleopController:
    """Manages per-arm teleop state and processes WebXR arm messages.

    Handles device-to-arm assignment, reference pose capture, IK target
    computation, and gripper delta tracking.
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

    @staticmethod
    def _new_arm_state():
        return {
            'xr_ref_pos': None,
            'xr_ref_rot_inv': None,
            'ref_ee_pos': None,
            'ref_ee_rot': None,
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

    def auto_assign_device(self, device_id):
        """Auto-assign device to first available arm (right, then left)."""
        if device_id in self.device_arm_map:
            return
        assigned_arms = set(self.device_arm_map.values())
        if 'right' not in assigned_arms:
            self.device_arm_map[device_id] = 'right'
        elif 'left' not in assigned_arms:
            self.device_arm_map[device_id] = 'left'
        # else: both arms taken, device won't get arm control

    def reset_device(self, device_id):
        """Reset arm refs on touch release (keep device-to-arm mapping)."""
        arm = self.device_arm_map.get(device_id)
        if arm:
            state = self.arm_state[arm]
            state['xr_ref_pos'] = None
            state['xr_ref_rot_inv'] = None
            state['ref_ee_pos'] = None
            state['ref_ee_rot'] = None
            state['control_active'] = False
            state['gripper_delta'] = 0.0

    def reset_all(self):
        """Emergency stop — clear everything."""
        for arm in ['right', 'left']:
            self.arm_state[arm] = self._new_arm_state()
        self.device_arm_map.clear()
        self.enabled_counts.clear()

    def process(self, device_id, pos, rot, arm_positions, gripper_delta=0.0):
        """Compute IK target from phone pose for the device's assigned arm.

        Args:
            device_id: Phone identifier string.
            pos: np.array[3] — phone position in robot frame.
            rot: scipy.spatial.transform.Rotation — phone orientation in robot frame.
            arm_positions: {'right': [6 floats], 'left': [6 floats]}
            gripper_delta: float — vertical swipe delta for gripper.
        """
        arm = self.device_arm_map.get(device_id)
        if arm is None:
            return  # Device not assigned to any arm

        state = self.arm_state[arm]
        current_arm_positions = arm_positions[arm]

        # On first touch: capture reference phone pose and current EE pose
        if state['xr_ref_pos'] is None:
            state['xr_ref_pos'] = pos.copy()
            state['xr_ref_rot_inv'] = rot.inv()

            if self.ik.available:
                ee_pos, ee_rot = self.ik.forward_kinematics(
                    arm, np.array(current_arm_positions))
                if ee_pos is not None:
                    state['ref_ee_pos'] = ee_pos.copy()
                    state['ref_ee_rot'] = ee_rot.copy()

        if state['ref_ee_pos'] is not None and self.ik.available:
            # Position: ref_ee + phone displacement
            delta_pos = pos - state['xr_ref_pos']
            target_ee_pos = state['ref_ee_pos'] + delta_pos

            # Orientation: relative phone rotation applied to ref EE orientation
            delta_rot = rot * state['xr_ref_rot_inv']
            target_ee_rot = delta_rot.as_matrix() @ state['ref_ee_rot']

            # Solve IK
            success, joints = self.ik.solve_ik(
                arm,
                target_ee_pos, target_ee_rot,
                seed=np.array(current_arm_positions),
                max_iter=30, dt=0.3, damping=1e-5)

            if success:
                # Verify IK solution actually reaches the target position
                solved_pos, _ = self.ik.forward_kinematics(arm, joints)
                if (solved_pos is not None and
                        np.linalg.norm(solved_pos - target_ee_pos) < ARM_IK_POS_TOL):
                    state['target_joints'] = joints
                    state['control_active'] = True

        state['gripper_delta'] = gripper_delta
