#!/usr/bin/env python3
"""
TidyBot2 Camera Pan-Tilt Demo

Demonstrates how to control the pan-tilt camera and capture images.
Takes a photo at center, pans right and tilts down, then takes another photo.

Topics used:
- /camera/pan_tilt_cmd (Float64MultiArray) - pan/tilt position commands [pan, tilt]
- /joint_states (JointState) - read current camera position
- /camera/color/image_raw (Image) - RGB camera feed

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run this demo
    ros2 run tidybot_bringup test_camera.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
import time
from pathlib import Path

from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray

# For saving images
try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False
    print("Note: cv_bridge/opencv not available. Image saving disabled.")
    print("      Install with: pip install opencv-python")


# Camera positions: (pan, tilt, name, capture?)
# pan: negative=left, positive=right
# tilt: negative=down, positive=up
POSITIONS = [
    (0.0, 0.0, "center", True),        # Start: capture
    (0.5, 0.0, "pan_right", False),    # Pan right
    (0.5, -0.4, "right_down", True),   # Tilt down: capture
]
SETTLE_TIME = 0.5  # seconds


class TestCamera(Node):
    """Camera pan-tilt demo with image capture."""

    def __init__(self):
        super().__init__('test_camera')

        # Publisher for pan-tilt commands
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # Subscriber for joint states
        self.joint_states = {}
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Image subscriber
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.latest_rgb = None

        if CV_AVAILABLE:
            self.cv_bridge = CvBridge()
            self.rgb_sub = self.create_subscription(
                Image, '/camera/color/image_raw', self.rgb_callback, qos
            )

        # State
        self.position_index = 0
        self.settled_time = None
        self.done = False

        # Output directory
        self.output_dir = Path(__file__).parent.parent / 'captures'
        self.output_dir.mkdir(exist_ok=True)

        # Control loop at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 Camera Pan-Tilt Demo')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Images saved to: {self.output_dir}')
        self.get_logger().info('')

    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            self.joint_states[name] = {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
            }

    def rgb_callback(self, msg: Image):
        if CV_AVAILABLE:
            try:
                self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, 'rgb8')
            except Exception as e:
                self.get_logger().warn(f'RGB conversion failed: {e}')

    def control_loop(self):
        if self.done:
            return

        if self.position_index >= len(POSITIONS):
            self.get_logger().info('')
            self.get_logger().info('=' * 50)
            self.get_logger().info('Demo complete!')
            self.get_logger().info('=' * 50)
            self.done = True
            return

        target_pan, target_tilt, name, should_capture = POSITIONS[self.position_index]

        # Send pan-tilt command
        msg = Float64MultiArray()
        msg.data = [target_pan, target_tilt]
        self.pan_tilt_pub.publish(msg)

        # Check if camera reached target
        current_pan = self.joint_states.get('camera_pan', {}).get('position', 0.0)
        current_tilt = self.joint_states.get('camera_tilt', {}).get('position', 0.0)

        at_target = abs(current_pan - target_pan) < 0.05 and abs(current_tilt - target_tilt) < 0.05

        if at_target:
            if self.settled_time is None:
                self.settled_time = time.time()
                self.get_logger().info(
                    f'Reached: {name} (pan={np.degrees(target_pan):.0f}, tilt={np.degrees(target_tilt):.0f})'
                )

            if time.time() - self.settled_time > SETTLE_TIME:
                if should_capture:
                    self.capture_image(name)
                self.position_index += 1
                self.settled_time = None
        else:
            self.settled_time = None

    def capture_image(self, name: str):
        if not CV_AVAILABLE or self.latest_rgb is None:
            self.get_logger().warn('Cannot capture image')
            return

        filename = self.output_dir / f'{name}.png'
        bgr = cv2.cvtColor(self.latest_rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(filename), bgr)
        self.get_logger().info(f'  Captured: {filename.name}')


def main(args=None):
    rclpy.init(args=args)
    node = TestCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
