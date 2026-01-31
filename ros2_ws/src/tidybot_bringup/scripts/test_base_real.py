#!/usr/bin/env python3
"""
Test Phoenix 6 Mobile Base.

Moves the base forward at 0.1 m/s for 1 second (~0.1m movement).

Usage:
    ros2 run tidybot_bringup test_base_real.py
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TestBase(Node):
    def __init__(self):
        super().__init__('test_base_real')

        self.odom_received = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.get_logger().info('Waiting for odometry...')

    def odom_callback(self, msg):
        if not self.odom_received:
            self.odom_received = True
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y
            self.get_logger().info(f'Connected! Start position: ({self.start_x:.3f}, {self.start_y:.3f})')

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def run_test(self):
        # Wait for connection
        timeout = 5.0
        start = time.time()
        while not self.odom_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.odom_received:
            self.get_logger().error('No odometry received! Is phoenix6_base_node running?')
            self.get_logger().error('Launch with: ros2 launch tidybot_bringup real.launch.py')
            return False

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('BASE TEST: Moving forward at 0.1 m/s for 1 second')
        self.get_logger().info('=' * 50)

        vel = Twist()
        vel.linear.x = 0.1  # Forward 0.1 m/s

        # Send velocity commands for 1 second
        start = time.time()
        while (time.time() - start) < 5.0:
            self.cmd_vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Stop
        vel.linear.x = 0.0
        self.cmd_vel_pub.publish(vel)

        # Calculate distance traveled
        import math
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        distance = math.sqrt(dx*dx + dy*dy)

        self.get_logger().info('')
        self.get_logger().info(f'Distance traveled: {distance:.3f} m')
        self.get_logger().info(f'End position: ({self.current_x:.3f}, {self.current_y:.3f})')
        self.get_logger().info('Base test complete!')

        return True


def main():
    rclpy.init()
    node = TestBase()
    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
