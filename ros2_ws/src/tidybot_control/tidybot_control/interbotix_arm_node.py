#!/usr/bin/env python3
"""
Interbotix WX200 Hardware Interface Node.

This node provides the SAME topic interface as the MuJoCo bridge,
but sends commands to real hardware via the Interbotix SDK.

Swap between sim and real by changing which node is launched:
- Simulation: mujoco_bridge_node (from tidybot_mujoco_bridge)
- Real hardware: interbotix_arm_node (this file)

Topics are IDENTICAL - student code doesn't change.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Interbotix SDK - only imported on real hardware
try:
    from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
    HAS_INTERBOTIX = True
except ImportError:
    HAS_INTERBOTIX = False


class InterbotixArmNode(Node):
    """
    Real hardware interface for Interbotix WX200 arm.

    Provides the SAME interface as MuJoCo bridge:
    - Subscribes to: /right_arm/joint_cmd, /left_arm/joint_cmd, etc.
    - Publishes to: /joint_states

    This allows student code to work identically on sim and real.
    """

    JOINT_NAMES = [
        'waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate'
    ]

    def __init__(self):
        super().__init__('interbotix_arm')

        # Parameters
        self.declare_parameter('arm_name', 'right')
        self.declare_parameter('robot_model', 'wx200')
        self.declare_parameter('publish_rate', 100.0)

        self.arm_name = self.get_parameter('arm_name').get_parameter_value().string_value
        robot_model = self.get_parameter('robot_model').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Check for Interbotix SDK
        if not HAS_INTERBOTIX:
            self.get_logger().error(
                'Interbotix SDK not installed! Install with:\n'
                '  pip install interbotix-xs-modules'
            )
            raise RuntimeError('Interbotix SDK not available')

        # Initialize real arm
        self.get_logger().info(f'Connecting to {robot_model} arm "{self.arm_name}"...')
        try:
            self.arm = InterbotixManipulatorXS(
                robot_model=robot_model,
                group_name='arm',
                gripper_name='gripper',
                robot_name=self.arm_name,
                init_node=False  # We already have a ROS2 node
            )
            self.get_logger().info('Connected to arm successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to arm: {e}')
            raise

        # Full joint names with arm prefix (matches simulation)
        self.full_joint_names = [f'{self.arm_name}_{j}' for j in self.JOINT_NAMES]
        self.full_joint_names += [f'{self.arm_name}_left_finger', f'{self.arm_name}_right_finger']

        # Publishers - SAME as MuJoCo bridge
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribers - SAME as MuJoCo bridge
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            f'/{self.arm_name}_arm/joint_cmd',
            self.joint_cmd_callback,
            10
        )
        self.gripper_cmd_sub = self.create_subscription(
            Float64MultiArray,
            f'/{self.arm_name}_gripper/cmd',
            self.gripper_cmd_callback,
            10
        )

        # Timer to publish joint states
        self.create_timer(1.0 / publish_rate, self.publish_joint_states)

        self.get_logger().info(f'{self.arm_name} arm hardware interface ready')

    def joint_cmd_callback(self, msg: Float64MultiArray):
        """
        Handle joint commands - send to real hardware.

        Same interface as MuJoCo bridge subscribes to.
        """
        if len(msg.data) != 5:
            self.get_logger().warn(f'Expected 5 joint values, got {len(msg.data)}')
            return

        try:
            # Send to real arm via Interbotix SDK
            self.arm.arm.set_joint_positions(list(msg.data))
        except Exception as e:
            self.get_logger().error(f'Failed to send joint command: {e}')

    def gripper_cmd_callback(self, msg: Float64MultiArray):
        """Handle gripper commands (normalized 0-1)."""
        if len(msg.data) < 1:
            return

        try:
            # Interbotix gripper uses effort-based control
            # 0 = open, 1 = closed
            if msg.data[0] > 0.5:
                self.arm.gripper.grasp()
            else:
                self.arm.gripper.release()
        except Exception as e:
            self.get_logger().error(f'Failed to send gripper command: {e}')

    def publish_joint_states(self):
        """
        Publish joint states from real hardware.

        Same format as MuJoCo bridge publishes.
        """
        try:
            # Read from real hardware
            positions = self.arm.arm.get_joint_positions()
            velocities = self.arm.arm.get_joint_velocities()

            # Get gripper state
            gripper_pos = self.arm.gripper.get_position()

            # Build JointState message - SAME format as simulation
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            # Arm joints
            for i, name in enumerate(self.full_joint_names[:5]):
                msg.name.append(name)
                msg.position.append(positions[i])
                msg.velocity.append(velocities[i] if i < len(velocities) else 0.0)
                msg.effort.append(0.0)

            # Gripper joints (simplified - both fingers same position)
            finger_pos = gripper_pos * 0.022  # Normalize to meters
            msg.name.append(f'{self.arm_name}_left_finger')
            msg.position.append(finger_pos)
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

            msg.name.append(f'{self.arm_name}_right_finger')
            msg.position.append(finger_pos)
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

            self.joint_state_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Failed to read joint states: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = InterbotixArmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
