#!/usr/bin/env python3
"""
TidyBot2 Arm Control Demo

Demonstrates how to control the bimanual WX200 arms using ROS2.
The arms move from home position to a forward reaching position.

Topics used:
- /right_arm/cmd (ArmCommand) - right arm joint position commands
- /left_arm/cmd (ArmCommand) - left arm joint position commands

ArmCommand message:
- joint_positions: [waist, shoulder, elbow, wrist_angle, wrist_rotate]
- duration: interpolation time (0 = immediate)

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run this demo
    ros2 run tidybot_bringup test_arms.py
"""

import rclpy
from rclpy.node import Node
import time

from tidybot_msgs.msg import ArmCommand


# Joint positions: [waist, shoulder, elbow, wrist_angle, wrist_rotate]
HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0]
FORWARD_POSITION = [0.0, 0.4, 0.5, -0.3, 0.0]  # Reaching forward

MOVE_DURATION = 2.0  # seconds


class TestArms(Node):
    """Arm control demo - move from home to forward position."""

    def __init__(self):
        super().__init__('test_arms')

        # Publishers for arm commands
        self.right_arm_pub = self.create_publisher(ArmCommand, '/right_arm/cmd', 10)
        self.left_arm_pub = self.create_publisher(ArmCommand, '/left_arm/cmd', 10)

        # State
        self.start_time = None
        self.command_sent = False
        self.done = False

        # Control loop at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 Arm Control Demo')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Moving arms from home to forward position')
        self.get_logger().info('')

    def send_arm_command(self, pub, positions, duration=0.0):
        """Send arm command with given joint positions."""
        cmd = ArmCommand()
        cmd.joint_positions = positions
        cmd.duration = duration
        pub.publish(cmd)

    def control_loop(self):
        if self.done:
            return

        now = time.time()

        if self.start_time is None:
            self.start_time = now

        elapsed = now - self.start_time

        # Send command once
        if not self.command_sent:
            self.get_logger().info('Sending arm commands...')
            self.send_arm_command(self.right_arm_pub, FORWARD_POSITION, duration=MOVE_DURATION)
            self.send_arm_command(self.left_arm_pub, FORWARD_POSITION, duration=MOVE_DURATION)
            self.command_sent = True

        # Wait for motion to complete
        if elapsed > MOVE_DURATION + 0.5:
            self.get_logger().info('')
            self.get_logger().info('=' * 50)
            self.get_logger().info('Done! Arms in forward position.')
            self.get_logger().info('=' * 50)
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = TestArms()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
