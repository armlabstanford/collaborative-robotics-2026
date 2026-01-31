#!/usr/bin/env python3
"""
Gripper Wrapper Node for TidyBot2.

Provides simulation-compatible gripper interface for real hardware.
Translates from:
    /right_gripper/cmd (Float64MultiArray, 0-1 normalized)
    /left_gripper/cmd (Float64MultiArray, 0-1 normalized)
To Interbotix SDK:
    /right_arm/commands/joint_single (JointSingleCommand, PWM effort)
    /left_arm/commands/joint_single (JointSingleCommand, PWM effort)

This allows the same user code to work for both simulation and real hardware.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from interbotix_xs_msgs.msg import JointSingleCommand


class GripperWrapperNode(Node):
    """Wrapper node to translate sim gripper commands to Interbotix SDK."""

    # PWM effort values for gripper control
    # These are tuned for WX250s grippers in PWM mode
    GRIPPER_PWM_OPEN = -200.0   # Negative = open
    GRIPPER_PWM_CLOSE = 250.0   # Positive = close

    def __init__(self):
        super().__init__('gripper_wrapper')

        # Publishers to Interbotix SDK
        self.right_gripper_pub = self.create_publisher(
            JointSingleCommand, '/right_arm/commands/joint_single', 10
        )
        self.left_gripper_pub = self.create_publisher(
            JointSingleCommand, '/left_arm/commands/joint_single', 10
        )

        # Subscribers - same topics as MuJoCo simulation
        self.right_gripper_sub = self.create_subscription(
            Float64MultiArray, '/right_gripper/cmd',
            lambda msg: self._gripper_callback(msg, 'right'), 10
        )
        self.left_gripper_sub = self.create_subscription(
            Float64MultiArray, '/left_gripper/cmd',
            lambda msg: self._gripper_callback(msg, 'left'), 10
        )

        self.get_logger().info('Gripper wrapper node started')
        self.get_logger().info('  Listening on /right_gripper/cmd and /left_gripper/cmd')
        self.get_logger().info('  Publishing to Interbotix SDK joint_single topics')

    def _gripper_callback(self, msg: Float64MultiArray, side: str):
        """
        Handle gripper command from simulation-compatible topic.

        Args:
            msg: Float64MultiArray with data[0] = normalized position (0=open, 1=closed)
            side: 'right' or 'left'
        """
        if len(msg.data) < 1:
            return

        # Normalize input to 0-1 range
        normalized = max(0.0, min(1.0, msg.data[0]))

        # Convert to PWM effort: 0 (open) -> negative PWM, 1 (closed) -> positive PWM
        # Linear interpolation between open and close PWM values
        pwm = self.GRIPPER_PWM_OPEN + normalized * (self.GRIPPER_PWM_CLOSE - self.GRIPPER_PWM_OPEN)

        # Create Interbotix command
        cmd = JointSingleCommand()
        if side == 'right':
            cmd.name = 'right_gripper'
            self.right_gripper_pub.publish(cmd)
        else:
            cmd.name = 'gripper'  # Left arm gripper is named 'gripper' in config
            self.left_gripper_pub.publish(cmd)

        cmd.cmd = float(pwm)

        # Publish to Interbotix SDK
        if side == 'right':
            self.right_gripper_pub.publish(cmd)
        else:
            self.left_gripper_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GripperWrapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
