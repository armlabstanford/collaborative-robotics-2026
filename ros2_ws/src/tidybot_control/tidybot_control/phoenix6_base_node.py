#!/usr/bin/env python3
import os
os.environ['CTR_TARGET'] = 'Hardware'  # MUST be before phoenix6 import

"""
Phoenix 6 Mobile Base ROS2 Node for TidyBot2.

This node wraps the Vehicle class from the tidybot2 repository and exposes
ROS2 topics for mobile base control. It provides the SAME interface as the
MuJoCo bridge, allowing code to work identically in simulation and on real hardware.

Topics (SAME as MuJoCo bridge):
- Subscribe: /cmd_vel (Twist) - velocity commands
- Subscribe: /base/target_pose (Pose2D) - position commands (go-to-goal)
- Publish: /odom (Odometry) - odometry feedback
- Publish: /base/goal_reached (Bool) - indicates when target pose is reached
- TF: odom -> base_link

Architecture:
- ROS2 executor handles pub/sub in main thread
- Dedicated control thread runs at 250 Hz (from Vehicle class)
- Thread-safe command queue for velocity/position commands

Prerequisites:
- Phoenix 6 library: pip install phoenix6
- Ruckig trajectory library: pip install ruckig
- tidybot2 repo available at TIDYBOT2_PATH or /home/locobot/tidybot2
- Real-time permissions for 250 Hz control (see /etc/security/limits.d/99-realtime.conf)

Usage:
    ros2 run tidybot_control phoenix6_base_node
"""

import sys
import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

# Add tidybot2 to path for importing Vehicle class
TIDYBOT2_PATH = os.environ.get('TIDYBOT2_PATH', '/home/locobot/tidybot2')
if TIDYBOT2_PATH not in sys.path:
    sys.path.insert(0, TIDYBOT2_PATH)

# Try to import Vehicle class
try:
    from base_controller import Vehicle, CommandType, FrameType
    HAS_PHOENIX6 = True
except ImportError as e:
    HAS_PHOENIX6 = False
    IMPORT_ERROR = str(e)


class Phoenix6BaseNode(Node):
    """
    ROS2 node for Phoenix 6 mobile base control.

    Provides the SAME interface as MuJoCo bridge for seamless sim-to-real transfer.
    """

    def __init__(self):
        super().__init__('phoenix6_base')

        # Check for Phoenix 6 / Vehicle class
        if not HAS_PHOENIX6:
            self.get_logger().error(
                f'Failed to import Vehicle class from tidybot2: {IMPORT_ERROR}\n'
                f'Ensure TIDYBOT2_PATH is set correctly (current: {TIDYBOT2_PATH})\n'
                f'Required packages: phoenix6, ruckig'
            )
            raise RuntimeError('Phoenix 6 / Vehicle class not available')

        # Declare parameters
        self.declare_parameter('max_linear_vel', 0.5)  # m/s
        self.declare_parameter('max_linear_vel_y', 0.5)  # m/s
        self.declare_parameter('max_angular_vel', 1.57)  # rad/s (~90 deg/s)
        self.declare_parameter('max_linear_accel', 0.25)  # m/s^2
        self.declare_parameter('max_angular_accel', 0.79)  # rad/s^2
        self.declare_parameter('publish_rate', 50.0)  # Hz for odometry/TF
        self.declare_parameter('position_tolerance', 0.02)  # meters
        self.declare_parameter('orientation_tolerance', 0.05)  # radians

        # Get parameters
        max_vx = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        max_vy = self.get_parameter('max_linear_vel_y').get_parameter_value().double_value
        max_vth = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        max_ax = self.get_parameter('max_linear_accel').get_parameter_value().double_value
        max_ath = self.get_parameter('max_angular_accel').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').get_parameter_value().double_value

        # Position control gains (matching MuJoCo bridge)
        self.kp_linear = 2.0
        self.kp_angular = 3.0
        self.max_linear_vel = max_vx
        self.max_angular_vel = max_vth

        # Initialize Vehicle (Phoenix 6 base controller)
        self.get_logger().info('Initializing Phoenix 6 mobile base...')
        try:
            self.vehicle = Vehicle(
                max_vel=(max_vx, max_vy, max_vth),
                max_accel=(max_ax, max_ax, max_ath),
                show_timing_warnings=False  # Suppress step time warnings in ROS2 context
            )
            self.get_logger().info('Phoenix 6 mobile base initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Vehicle: {e}')
            raise

        # Control state
        self.lock = threading.Lock()
        self.position_control_mode = False
        self.target_pose = None  # Pose2D (x, y, theta) in robot frame
        self.cmd_vel = Twist()
        self.last_goal_reached = False

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers - SAME as MuJoCo bridge
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/base/goal_reached', 10)

        # Subscribers - SAME as MuJoCo bridge
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.target_pose_sub = self.create_subscription(
            Pose2D, '/base/target_pose', self.target_pose_callback, 10
        )

        # Timer for publishing odometry and TF
        self.create_timer(1.0 / self.publish_rate, self.publish_callback)

        # Timer for sending commands to vehicle (at policy control rate)
        # The Vehicle class has its own 250 Hz control loop, but we send commands
        # at a lower rate to avoid overwhelming the command queue
        self.create_timer(0.1, self.control_callback)  # 10 Hz command rate

        # Start the Vehicle control loop (runs in separate thread at 250 Hz)
        self.vehicle.start_control()

        self.get_logger().info('Phoenix 6 base node ready')
        self.get_logger().info(f'  Max velocity: ({max_vx}, {max_vy}, {max_vth}) m/s, rad/s')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands for the mobile base."""
        with self.lock:
            # Velocity commands cancel position control mode
            self.position_control_mode = False
            self.target_pose = None
            self.cmd_vel = msg

    def target_pose_callback(self, msg: Pose2D):
        """
        Handle position target for the mobile base (go-to-goal).

        Target coordinates are in robot-forward-aligned frame where:
        - x is forward (direction camera/arms face)
        - y is left
        - theta is rotation about z-axis

        This matches the MuJoCo bridge coordinate transform (lines 263-278).
        """
        with self.lock:
            self.position_control_mode = True
            # Store target in world frame, converting from robot-forward frame
            # Robot faces -Y at theta=0 in world frame, so we rotate by -pi/2:
            # x_world = y_input, y_world = -x_input
            self.target_pose = Pose2D()
            self.target_pose.x = msg.y
            self.target_pose.y = -msg.x
            self.target_pose.theta = msg.theta
            self.last_goal_reached = False
            self.get_logger().info(
                f'Target: forward={msg.x:.2f}m -> world ({self.target_pose.x:.2f}, {self.target_pose.y:.2f})'
            )

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def compute_position_control_velocity(self, x, y, th):
        """
        Compute velocity commands to reach target pose.

        This matches the MuJoCo bridge implementation (lines 347-414).
        """
        if self.target_pose is None:
            return 0.0, 0.0, 0.0

        # Target pose
        tx, ty, tth = self.target_pose.x, self.target_pose.y, self.target_pose.theta

        # Position error in world frame
        dx = tx - x
        dy = ty - y
        distance = math.sqrt(dx * dx + dy * dy)

        # Angle to target (in world frame)
        angle_to_target = math.atan2(dy, dx)

        # Heading error (difference between current heading and angle to target)
        # The robot model faces -Y at theta=0, so actual heading is theta - pi/2
        actual_heading = th - math.pi / 2
        heading_error = self.normalize_angle(angle_to_target - actual_heading)

        # Final orientation error
        orientation_error = self.normalize_angle(tth - th)

        # Check if we've reached the goal
        at_position = distance < self.position_tolerance
        at_orientation = abs(orientation_error) < self.orientation_tolerance

        if at_position and at_orientation:
            # Goal reached
            if not self.last_goal_reached:
                goal_msg = Bool()
                goal_msg.data = True
                self.goal_reached_pub.publish(goal_msg)
                self.get_logger().info('Goal reached!')
                self.last_goal_reached = True
            self.position_control_mode = False
            self.target_pose = None
            return 0.0, 0.0, 0.0

        # Two-phase control: first rotate to face target, then drive + rotate to final
        if not at_position:
            if abs(heading_error) > 0.3:  # ~17 degrees - need to rotate first
                # Rotate in place to face target
                vx = 0.0
                vy = 0.0
                vth = self.kp_angular * heading_error
            else:
                # Drive toward target while correcting heading
                vx = self.kp_linear * distance
                vy = 0.0
                vth = self.kp_angular * heading_error
        else:
            # Phase 2: At position, rotate to final orientation
            vx = 0.0
            vy = 0.0
            vth = self.kp_angular * orientation_error

        # Apply velocity limits
        vx = np.clip(vx, -self.max_linear_vel, self.max_linear_vel)
        vy = np.clip(vy, -self.max_linear_vel, self.max_linear_vel)
        vth = np.clip(vth, -self.max_angular_vel, self.max_angular_vel)

        return vx, vy, vth

    def control_callback(self):
        """Send velocity commands to the Vehicle (called at 10 Hz)."""
        with self.lock:
            # Get current pose from vehicle
            x, y, th = self.vehicle.x[0], self.vehicle.x[1], self.vehicle.x[2]

            if self.position_control_mode:
                # Compute velocity from position controller
                vx, vy, vth = self.compute_position_control_velocity(x, y, th)
            else:
                # Use cmd_vel directly
                vx = self.cmd_vel.linear.x
                vy = self.cmd_vel.linear.y
                vth = self.cmd_vel.angular.z

            # Send to vehicle in local frame
            # Transform from ROS convention to vehicle local frame
            # ROS: +x forward, +y left, +z up
            # Vehicle local: same convention
            velocity = np.array([vx, vy, vth])
            self.vehicle.set_target_velocity(velocity, frame='local')

    def publish_callback(self):
        """Publish odometry and TF."""
        now = self.get_clock().now().to_msg()

        with self.lock:
            # Get state from vehicle
            x, y, th = self.vehicle.x[0], self.vehicle.x[1], self.vehicle.x[2]
            vx, vy, vth = self.vehicle.dx[0], self.vehicle.dx[1], self.vehicle.dx[2]

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # Apply pi/2 yaw offset to match URDF mesh convention.
        # The Vehicle class uses theta=0 for forward (+X), but the URDF mesh
        # faces -Y at joint_th=0 (same as MuJoCo model), so joint_th=pi/2
        # means forward. Adding pi/2 aligns the RViz visualization with the
        # actual robot heading.
        th_odom = th + math.pi / 2

        # Convert yaw to quaternion
        cy = math.cos(th_odom * 0.5)
        sy = math.sin(th_odom * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        # Publish TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy

        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Shutting down Phoenix 6 base node...')
        try:
            self.vehicle.stop_control()
        except Exception as e:
            self.get_logger().warn(f'Error stopping vehicle: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Phoenix6BaseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
