#!/usr/bin/env python3
"""
TidyBot2 Base Movement Demo

Demonstrates how to control the mobile base using ROS2 velocity commands.
The robot will drive forward a specified distance, then stop.

Topics used:
- /cmd_vel (geometry_msgs/Twist) - velocity commands to the base

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run this demo
    ros2 run tidybot_bringup test_base.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


# Configuration
LINEAR_SPEED = 0.3       # m/s
TARGET_DISTANCE = 1.0    # meters


class TestBase(Node):
    """Simple base movement demo - drive forward then stop."""

    def __init__(self):
        super().__init__('test_base')

        # Publisher for base velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State tracking
        self.start_time = None
        self.distance_traveled = 0.0
        self.done = False

        # Control loop at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 Base Movement Demo')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Target: {TARGET_DISTANCE}m at {LINEAR_SPEED}m/s')
        self.get_logger().info('')

    def control_loop(self):
        if self.done:
            return

        # Initialize timing
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info('Starting movement...')

        twist = Twist()

        if self.distance_traveled < TARGET_DISTANCE:
            # Drive forward
            twist.linear.x = LINEAR_SPEED
            self.distance_traveled += LINEAR_SPEED * 0.02  # dt = 0.02s

            # Log progress every 0.25m
            if int(self.distance_traveled * 4) > int((self.distance_traveled - LINEAR_SPEED * 0.02) * 4):
                self.get_logger().info(f'Distance: {self.distance_traveled:.2f}m / {TARGET_DISTANCE}m')
        else:
            # Stop
            twist.linear.x = 0.0
            elapsed = time.time() - self.start_time
            self.get_logger().info('')
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'Done! Traveled {self.distance_traveled:.2f}m in {elapsed:.1f}s')
            self.get_logger().info('=' * 50)
            self.done = True

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TestBase()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before shutting down
        stop = Twist()
        node.cmd_vel_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
