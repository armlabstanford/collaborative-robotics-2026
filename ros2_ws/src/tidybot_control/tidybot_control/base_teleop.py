"""Base teleop: tilt-based velocity control from phone orientation.

Converts phone tilt (WebXR) into robot base velocity commands [vx, vy, wz].

Linear velocity: tilt forward/back and left/right → proportional speed.
Heading: phone yaw directly sets robot target heading (1:1 position control).
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation as R


# Base velocity control parameters (tilt-based)
BASE_MAX_LINEAR_VEL = 0.5        # m/s cap
BASE_MAX_ANGULAR_VEL = 1.0       # rad/s cap
BASE_MAX_TILT_ANGLE = math.pi / 4  # 45 deg tilt = full speed
BASE_TILT_DEADZONE = 0.05        # radians (~3 deg) — ignore small tilts

# Heading (yaw) position control parameters
HEADING_P_GAIN = 3.0             # proportional gain for heading tracking
HEADING_DEADZONE = 0.03          # radians (~2 deg) — ignore tiny heading errors


def _tilt_to_vel(angle, deadzone, max_angle, max_vel):
    """Map a tilt angle to a velocity with deadzone and clamping."""
    if abs(angle) < deadzone:
        return 0.0
    frac = np.clip(angle / max_angle, -1.0, 1.0)
    return frac * max_vel


class BaseTeleopController:
    """Processes WebXR orientation into base velocity commands.

    Stores per-device reference orientation and computes tilt-based
    velocity from the delta.  Heading uses position control: the phone
    yaw sets the robot target heading (1:1), and a P controller drives
    angular velocity to close the error.
    """

    def __init__(self):
        self.base_xr_ref_rot_inv = {}   # {device_id: rot_inv}
        self.base_cmd_vel = np.array([0.0, 0.0, 0.0])  # [vx, vy, wz]
        self.base_control_active = False
        self.base_control_device = None
        self.robot_heading = 0.0        # estimated heading (integrated wz)
        self._heading_at_touch = 0.0    # heading when touch started

    def update_heading(self, wz, dt):
        """Integrate angular velocity to track estimated robot heading."""
        self.robot_heading += wz * dt

    def reset_device(self, device_id):
        """Clear reference for a device (on touch release or disconnect)."""
        self.base_xr_ref_rot_inv.pop(device_id, None)
        if self.base_control_device == device_id:
            self.base_cmd_vel[:] = 0.0
            self.base_control_active = False
            self.base_control_device = None

    def reset_all(self):
        """Emergency stop — clear everything."""
        self.base_cmd_vel[:] = 0.0
        self.base_control_active = False
        self.base_control_device = None
        self.base_xr_ref_rot_inv.clear()

    def process(self, device_id, rot):
        """Compute base velocity from phone orientation.

        Args:
            device_id: Phone identifier string.
            rot: scipy.spatial.transform.Rotation — phone orientation in robot frame.
        """
        if device_id not in self.base_xr_ref_rot_inv:
            self.base_xr_ref_rot_inv[device_id] = rot.inv()
            self._heading_at_touch = self.robot_heading

        delta_rot = rot * self.base_xr_ref_rot_inv[device_id]
        rx, ry, rz = delta_rot.as_euler('xyz')

        # Linear velocity from tilt
        vx = _tilt_to_vel(ry, BASE_TILT_DEADZONE,
                          BASE_MAX_TILT_ANGLE, BASE_MAX_LINEAR_VEL)
        vy = _tilt_to_vel(-rx, BASE_TILT_DEADZONE,
                          BASE_MAX_TILT_ANGLE, BASE_MAX_LINEAR_VEL)

        # Heading: phone yaw = target heading (position control)
        target_heading = self._heading_at_touch + rz
        heading_error = target_heading - self.robot_heading
        # Wrap to [-pi, pi]
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        if abs(heading_error) < HEADING_DEADZONE:
            wz = 0.0
        else:
            wz = np.clip(HEADING_P_GAIN * heading_error,
                         -BASE_MAX_ANGULAR_VEL, BASE_MAX_ANGULAR_VEL)

        self.base_cmd_vel = np.array([vx, vy, wz])
        self.base_control_active = True
        self.base_control_device = device_id
