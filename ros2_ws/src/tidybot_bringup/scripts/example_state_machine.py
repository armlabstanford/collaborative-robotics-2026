#!/usr/bin/env python3
"""
TidyBot2 State Machine Example

Demonstrates a simple state machine pattern for sequencing robot actions:
1. Move base forward 0.5m
2. Take a picture
3. Move arms to a downward position
4. Close grippers

This pattern is useful for building more complex robot behaviors.

Topics used:
- /cmd_vel (Twist) - base velocity
- /camera/color/image_raw (Image) - camera feed
- /right_arm/cmd, /left_arm/cmd (ArmCommand) - arm positions
- /right_gripper/cmd, /left_gripper/cmd (Float64MultiArray) - gripper control

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run this demo
    ros2 run tidybot_bringup example_state_machine.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import time
from pathlib import Path
from enum import Enum, auto

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from tidybot_msgs.msg import ArmCommand

# For saving images
try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class State(Enum):
    """States for our simple task."""
    MOVE_FORWARD = auto()
    TAKE_PICTURE = auto()
    MOVE_ARMS = auto()
    CLOSE_GRIPPERS = auto()
    DONE = auto()


# Configuration
MOVE_DISTANCE = 0.5      # meters
MOVE_SPEED = 0.2         # m/s
ARM_MOVE_DURATION = 2.0  # seconds
GRIPPER_CLOSE_TIME = 1.0 # seconds

# Arm positions: [waist, shoulder, elbow, wrist_angle, wrist_rotate]
ARM_DOWN_POSITION = [0.0, 0.8, 0.4, 0.0, 0.0]  # Arms reaching down/forward


class StateMachineExample(Node):
    """Example state machine combining base, camera, arms, and grippers."""

    def __init__(self):
        super().__init__('state_machine_example')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.right_arm_pub = self.create_publisher(ArmCommand, '/right_arm/cmd', 10)
        self.left_arm_pub = self.create_publisher(ArmCommand, '/left_arm/cmd', 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, '/right_gripper/cmd', 10)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, '/left_gripper/cmd', 10)

        # Image subscriber
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.latest_rgb = None
        if CV_AVAILABLE:
            self.cv_bridge = CvBridge()
            self.rgb_sub = self.create_subscription(
                Image, '/camera/color/image_raw', self.rgb_callback, qos
            )

        # State machine variables
        self.state = State.MOVE_FORWARD
        self.state_start_time = None
        self.distance_traveled = 0.0

        # Output directory for images
        self.output_dir = Path(__file__).parent.parent / 'captures'
        self.output_dir.mkdir(exist_ok=True)

        # Control loop at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 State Machine Example')
        self.get_logger().info('=' * 50)
        self.get_logger().info('States: MOVE -> PICTURE -> ARMS -> GRIPPERS -> DONE')
        self.get_logger().info('')

    def rgb_callback(self, msg: Image):
        if CV_AVAILABLE:
            try:
                self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'rgb8')
            except Exception:
                pass

    def transition_to(self, new_state: State):
        """Transition to a new state."""
        self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_start_time = time.time()

    def control_loop(self):
        """Main control loop - runs the state machine."""
        now = time.time()

        # Initialize state timing
        if self.state_start_time is None:
            self.state_start_time = now
            self.get_logger().info(f'Starting state: {self.state.name}')

        elapsed = now - self.state_start_time

        # ==================== STATE: MOVE FORWARD ====================
        if self.state == State.MOVE_FORWARD:
            twist = Twist()

            if self.distance_traveled < MOVE_DISTANCE:
                twist.linear.x = MOVE_SPEED
                self.distance_traveled += MOVE_SPEED * 0.02  # dt = 0.02s
                self.cmd_vel_pub.publish(twist)
            else:
                # Stop and transition
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f'  Moved {self.distance_traveled:.2f}m')
                self.transition_to(State.TAKE_PICTURE)

        # ==================== STATE: TAKE PICTURE ====================
        elif self.state == State.TAKE_PICTURE:
            # Wait a moment for camera to stabilize
            if elapsed > 0.5:
                if CV_AVAILABLE and self.latest_rgb is not None:
                    filename = self.output_dir / 'state_machine_capture.png'
                    bgr = cv2.cvtColor(self.latest_rgb, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(str(filename), bgr)
                    self.get_logger().info(f'  Saved: {filename.name}')
                else:
                    self.get_logger().info('  (camera not available)')
                self.transition_to(State.MOVE_ARMS)

        # ==================== STATE: MOVE ARMS ====================
        elif self.state == State.MOVE_ARMS:
            # Send arm command once at start of state
            if elapsed < 0.1:
                cmd = ArmCommand()
                cmd.joint_positions = ARM_DOWN_POSITION
                cmd.duration = ARM_MOVE_DURATION
                self.right_arm_pub.publish(cmd)
                self.left_arm_pub.publish(cmd)
                self.get_logger().info('  Moving arms down...')

            # Wait for arms to finish moving
            if elapsed > ARM_MOVE_DURATION + 0.5:
                self.transition_to(State.CLOSE_GRIPPERS)

        # ==================== STATE: CLOSE GRIPPERS ====================
        elif self.state == State.CLOSE_GRIPPERS:
            # Send gripper command once at start
            if elapsed < 0.1:
                msg = Float64MultiArray()
                msg.data = [1.0]  # 1.0 = closed, 0.0 = open
                self.right_gripper_pub.publish(msg)
                self.left_gripper_pub.publish(msg)
                self.get_logger().info('  Closing grippers...')

            # Wait for grippers to close
            if elapsed > GRIPPER_CLOSE_TIME:
                self.transition_to(State.DONE)

        # ==================== STATE: DONE ====================
        elif self.state == State.DONE:
            if elapsed < 0.1:  # Only log once
                self.get_logger().info('')
                self.get_logger().info('=' * 50)
                self.get_logger().info('State machine complete!')
                self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineExample()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop base on exit
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
