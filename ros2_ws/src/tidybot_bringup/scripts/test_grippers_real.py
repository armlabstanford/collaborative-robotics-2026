#!/usr/bin/env python3
"""
Test Interbotix Grippers.

Opens and closes both grippers.

Usage:
    ros2 run tidybot_bringup test_grippers_real.py
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class TestGrippers(Node):
    """
    Test grippers using simulation-compatible interface.

    Uses the same /right_gripper/cmd and /left_gripper/cmd topics
    as the MuJoCo simulation, so this script works for both sim and real.
    """

    def __init__(self):
        super().__init__('test_grippers_real')

        self.joint_states_received = False

        # Publishers - same interface as MuJoCo simulation
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper/cmd', 10
        )
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, '/left_gripper/cmd', 10
        )

        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        self.get_logger().info('Waiting for joint states...')

    def joint_callback(self, msg):
        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info('Connected!')

    def send_gripper_command(self, value, duration=1.0):
        """Send gripper command (0.0 = open, 1.0 = closed)."""
        msg = Float64MultiArray()
        msg.data = [float(value)]

        for _ in range(int(duration * 20)):
            self.right_gripper_pub.publish(msg)
            self.left_gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_test(self):
        # Wait for connection
        timeout = 5.0
        start = time.time()
        while not self.joint_states_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.joint_states_received:
            self.get_logger().error('No joint states received!')
            return False

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('GRIPPER TEST')
        self.get_logger().info('=' * 50)

        self.get_logger().info('Opening grippers...')
        self.send_gripper_command(0.0, duration=1.5)

        self.get_logger().info('Closing grippers...')
        self.send_gripper_command(1.0, duration=1.5)

        self.get_logger().info('Opening grippers again...')
        self.send_gripper_command(0.0, duration=1.5)

        self.get_logger().info('')
        self.get_logger().info('Gripper test complete!')

        return True


def main():
    rclpy.init()
    node = TestGrippers()
    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
