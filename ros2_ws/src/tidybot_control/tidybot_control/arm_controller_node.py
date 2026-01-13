#!/usr/bin/env python3
"""
Arm Controller Node for TidyBot2.

Provides high-level arm control:
- Accepts ArmCommand messages with target positions and duration
- Interpolates trajectories smoothly
- Enforces joint limits
- Publishes joint commands to the MuJoCo bridge
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Import custom messages
from tidybot_msgs.msg import ArmCommand, GripperCommand


class ArmControllerNode(Node):
    """High-level arm controller with trajectory interpolation."""

    # Joint limits (from tidybot_wx200_bimanual.xml)
    JOINT_LIMITS = {
        'waist': (-3.14159, 3.14159),
        'shoulder': (-1.8849, 1.9722),
        'elbow': (-1.8849, 1.6232),
        'wrist_angle': (-1.7453, 2.1468),
        'wrist_rotate': (-3.14159, 3.14159),
    }

    def __init__(self):
        super().__init__('arm_controller')

        # Declare parameters
        self.declare_parameter('arm_name', 'right')
        self.declare_parameter('control_rate', 50.0)

        self.arm_name = self.get_parameter('arm_name').get_parameter_value().string_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value

        # Joint names for this arm
        self.joint_names = [
            f'{self.arm_name}_waist',
            f'{self.arm_name}_shoulder',
            f'{self.arm_name}_elbow',
            f'{self.arm_name}_wrist_angle',
            f'{self.arm_name}_wrist_rotate',
        ]

        # State variables
        self.current_positions = np.zeros(5)
        self.target_positions = np.zeros(5)
        self.start_positions = np.zeros(5)
        self.trajectory_duration = 0.0
        self.trajectory_time = 0.0
        self.in_trajectory = False

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self.arm_name}_arm/joint_cmd',
            10
        )

        # Subscribers
        self.arm_cmd_sub = self.create_subscription(
            ArmCommand,
            f'/{self.arm_name}_arm/cmd',
            self.arm_cmd_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Control timer
        control_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(control_period, self.control_callback)

        self.get_logger().info(f'{self.arm_name.capitalize()} arm controller initialized')

    def joint_state_callback(self, msg: JointState):
        """Update current joint positions from joint states."""
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                self.current_positions[i] = msg.position[idx]

    def arm_cmd_callback(self, msg: ArmCommand):
        """Handle incoming arm commands."""
        if len(msg.joint_positions) != 5:
            self.get_logger().warn(f'ArmCommand has {len(msg.joint_positions)} positions, expected 5')
            return

        # Clamp to joint limits
        target = np.array(msg.joint_positions)
        for i, (_, (low, high)) in enumerate(self.JOINT_LIMITS.items()):
            target[i] = np.clip(target[i], low, high)

        if msg.duration > 0:
            # Start interpolated trajectory
            self.start_positions = self.current_positions.copy()
            self.target_positions = target
            self.trajectory_duration = msg.duration
            self.trajectory_time = 0.0
            self.in_trajectory = True
            self.get_logger().debug(f'Starting trajectory to {target} over {msg.duration}s')
        else:
            # Immediate command
            self.target_positions = target
            self.in_trajectory = False
            self.publish_command(target)

    def control_callback(self):
        """Execute trajectory interpolation."""
        if not self.in_trajectory:
            return

        dt = 1.0 / self.control_rate
        self.trajectory_time += dt

        if self.trajectory_time >= self.trajectory_duration:
            # Trajectory complete
            self.in_trajectory = False
            self.publish_command(self.target_positions)
            return

        # Smooth interpolation using cosine easing
        t = self.trajectory_time / self.trajectory_duration
        # Cosine easing: slow start and end, fast middle
        alpha = (1 - np.cos(t * np.pi)) / 2

        interpolated = self.start_positions + alpha * (self.target_positions - self.start_positions)
        self.publish_command(interpolated)

    def publish_command(self, positions: np.ndarray):
        """Publish joint command to bridge."""
        msg = Float64MultiArray()
        msg.data = positions.tolist()
        self.joint_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ArmControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
