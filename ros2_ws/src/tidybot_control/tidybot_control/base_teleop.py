"""Base teleop: position-based control from phone pose (like arm teleop).

The phone acts as a physical proxy for the robot base:
  - Phone XY displacement → robot XY position target (1:1)
  - Phone yaw → robot heading target (1:1)
  - P controller converts position/heading error → cmd_vel
  - Odom feedback prevents drift
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation as R


# Position control gains
POS_P_GAIN = 2.0                 # proportional gain for XY position
HEADING_P_GAIN = 3.0             # proportional gain for heading
BASE_MAX_LINEAR_VEL = 0.5        # m/s cap
BASE_MAX_ANGULAR_VEL = 1.0       # rad/s cap
POS_DEADZONE = 0.01              # meters — ignore tiny position errors
HEADING_DEADZONE = 0.03          # radians (~2 deg) — ignore tiny heading errors


class BaseTeleopController:
    """Position-based base teleop using phone displacement + odom feedback.

    On touch-start, captures the phone pose and robot pose (from odom).
    Phone displacement from reference → robot target pose.
    P controller drives cmd_vel to close the error each tick.
    """

    def __init__(self):
        self.base_cmd_vel = np.array([0.0, 0.0, 0.0])  # [vx, vy, wz]
        self.base_control_active = False
        self.base_control_device = None

        # Odom state (updated externally via update_odom)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        # Per-device reference state
        self._ref = {}  # {device_id: {phone_pos, phone_rot_inv, robot_pose}}

    def update_odom(self, x, y, theta):
        """Called from the odom subscriber callback."""
        self.odom_x = x
        self.odom_y = y
        self.odom_theta = theta

    def reset_device(self, device_id):
        """Clear reference for a device (on touch release or disconnect)."""
        self._ref.pop(device_id, None)
        if self.base_control_device == device_id:
            self.base_cmd_vel[:] = 0.0
            self.base_control_active = False
            self.base_control_device = None

    def reset_all(self):
        """Emergency stop — clear everything."""
        self.base_cmd_vel[:] = 0.0
        self.base_control_active = False
        self.base_control_device = None
        self._ref.clear()

    def process(self, device_id, pos, rot):
        """Compute base velocity from phone pose + odom feedback.

        Args:
            device_id: Phone identifier string.
            pos: np.array[3] — phone position in robot frame.
            rot: scipy.spatial.transform.Rotation — phone orientation in robot frame.
        """
        # Capture reference on first touch
        if device_id not in self._ref:
            self._ref[device_id] = {
                'phone_pos': pos.copy(),
                'phone_rot_inv': rot.inv(),
                'robot_x': self.odom_x,
                'robot_y': self.odom_y,
                'robot_theta': self.odom_theta,
            }

        ref = self._ref[device_id]

        # Phone displacement since touch-start (in session frame ≈ odom frame)
        delta_pos = pos - ref['phone_pos']
        delta_rot = rot * ref['phone_rot_inv']
        _, _, delta_yaw = delta_rot.as_euler('xyz')

        # Target pose in odom frame: ref + phone displacement directly
        # Session frame and odom frame share the same orientation,
        # so no rotation needed — 1:1 world-frame mapping.
        target_x = ref['robot_x'] + delta_pos[0]
        target_y = ref['robot_y'] + delta_pos[1]
        target_theta = ref['robot_theta'] + delta_yaw

        # Position error in odom frame
        err_x_odom = target_x - self.odom_x
        err_y_odom = target_y - self.odom_y

        # Transform position error into robot body frame for cmd_vel
        cos_cur = math.cos(self.odom_theta)
        sin_cur = math.sin(self.odom_theta)
        err_x_body = cos_cur * err_x_odom + sin_cur * err_y_odom
        err_y_body = -sin_cur * err_x_odom + cos_cur * err_y_odom

        # Heading error (wrapped to [-pi, pi])
        heading_error = target_theta - self.odom_theta
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # P controller → cmd_vel (body frame)
        pos_err_mag = math.sqrt(err_x_body**2 + err_y_body**2)
        if pos_err_mag < POS_DEADZONE:
            vx, vy = 0.0, 0.0
        else:
            vx = np.clip(POS_P_GAIN * err_x_body,
                         -BASE_MAX_LINEAR_VEL, BASE_MAX_LINEAR_VEL)
            vy = np.clip(POS_P_GAIN * err_y_body,
                         -BASE_MAX_LINEAR_VEL, BASE_MAX_LINEAR_VEL)

        if abs(heading_error) < HEADING_DEADZONE:
            wz = 0.0
        else:
            wz = np.clip(HEADING_P_GAIN * heading_error,
                         -BASE_MAX_ANGULAR_VEL, BASE_MAX_ANGULAR_VEL)

        self.base_cmd_vel = np.array([vx, vy, wz])
        self.base_control_active = True
        self.base_control_device = device_id
