# TidyBot2 Remote Control Setup

This guide explains how to control TidyBot2 remotely over the network using native ROS2.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    MINI PC (TidyBot2 Robot)                     │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                   ROS2 Network (DDS)                      │  │
│  │  /cmd_vel  /joint_states  /odom  /camera/*  /arm/cmd     │  │
│  └──────────────────────────────────────────────────────────┘  │
│        ▲              ▲              ▲              ▲          │
│        │              │              │              │          │
│  ┌─────┴─────┐  ┌─────┴─────┐  ┌─────┴─────┐  ┌─────┴─────┐  │
│  │ Phoenix6  │  │ Interbotix│  │ RealSense │  │ Pan-Tilt  │  │
│  │ Base Node │  │ Arm Nodes │  │ Camera    │  │ Camera    │  │
│  └───────────┘  └───────────┘  └───────────┘  └───────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              │
                     WiFi / Ethernet
                              │
┌─────────────────────────────────────────────────────────────────┐
│                  YOUR MACHINE (Remote Client)                    │
│                                                                 │
│  ROS2 Humble + Same Domain ID = Direct topic access             │
│                                                                 │
│  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...            │
│  ros2 topic echo /joint_states                                  │
│  ros2 run your_package your_node                                │
└─────────────────────────────────────────────────────────────────┘
```

## Key Principle: Native ROS2 Networking

You run **real ROS2 nodes** on your machine, not a custom client library. This:
- Teaches proper ROS2 usage
- Uses standard ROS2 tools (`ros2 topic`, `ros2 run`, `rviz2`)
- Code written for simulation works unchanged on real robot
- Skills transfer to any ROS2 robot

## Prerequisites

### On Your Machine (Client)

1. **ROS2 Humble installed** (required)
   ```bash
   # Ubuntu 22.04
   sudo apt install ros-humble-desktop
   ```

2. **tidybot_msgs package** (for custom message types)
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Stanford-ARM-Lab/collaborative-robotics-2026.git
   cd ~/ros2_ws
   colcon build --packages-select tidybot_msgs
   source install/setup.bash
   ```

3. **Cyclone DDS** (recommended for easier networking)
   ```bash
   sudo apt install ros-humble-rmw-cyclonedds-cpp
   ```

### Network Requirements

- Client and robot on the **same network** (same WiFi or Ethernet subnet)
- If multicast is blocked, use FastDDS Discovery Server mode (see below)

## Quick Start

### Step 1: Start Robot (on TidyBot2 mini PC)

```bash
# SSH into robot
ssh locobot@192.169.1.7 # NUC Ip address # ssh password: locobot

# Set up environment
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Launch robot
cd ~/collaborative-robotics-2026/ros2_ws
source install/setup.bash
ros2 launch tidybot_bringup robot.launch.py
```

### Step 2: Connect Client (on your machine)

```bash
# Set up environment (must match robot!)
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source your workspace
source ~/ros2_ws/install/setup.bash

# Test connection
ros2 topic list
```

You should see topics like:
```
/cmd_vel
/odom
/joint_states
/right_arm/command
/left_arm/command
/camera/color/image_raw
...
```

If you don't see ros2 topics, try disabling your local network firewall (as well as on the NUC):

```
sudo ufw disable
```

### Step 3: Control the Robot

```bash
# Move the base forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -r 10

# Read joint states
ros2 topic echo /joint_states

# Send arm command (using test script)
ros2 run tidybot_bringup test_arms_sim.py
```

## Network Configuration Options

### Option A: Cyclone DDS with Multicast (Simplest)

Works when robot and client are on the same subnet and multicast is allowed.

**Robot:**
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

**Client:**
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

### Option B: FastDDS Discovery Server (Works Across Subnets)

Use when multicast is blocked or machines are on different subnets.

**Robot (start discovery server first):**
```bash
# Terminal 1: Start discovery server
fastdds discovery --server-id 0 --port 11811

# Terminal 2: Launch robot
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix tidybot_bringup)/share/tidybot_bringup/config/fastdds_robot.xml
export ROS_DOMAIN_ID=42
ros2 launch tidybot_bringup robot.launch.py
```

**Client:**
```bash
# Edit config to set robot IP
# In fastdds_client.xml, replace 192.168.1.100 with actual robot IP

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=<path_to>/fastdds_client.xml
export ROS_DOMAIN_ID=42
```

### Option C: Zenoh Bridge (Enterprise-Grade)

For complex network topologies. See [Zenoh documentation](https://zenoh.io/docs/getting-started/first-app/).

## ROS2 Topics Reference

### Mobile Base

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (linear.x, linear.y, angular.z) |
| `/base/target_pose` | `geometry_msgs/Pose2D` | Go-to-goal position (x, y, theta) |
| `/odom` | `nav_msgs/Odometry` | Odometry feedback |
| `/base/goal_reached` | `std_msgs/Bool` | True when target pose reached |

### Arms

| Topic | Type | Description |
|-------|------|-------------|
| `/right_arm/command` | `tidybot_msgs/ArmCommand` | Right arm joint positions + duration |
| `/left_arm/command` | `tidybot_msgs/ArmCommand` | Left arm joint positions + duration |
| `/right_gripper/command` | `tidybot_msgs/GripperCommand` | Right gripper (0=open, 1=closed) |
| `/left_gripper/command` | `tidybot_msgs/GripperCommand` | Left gripper |
| `/joint_states` | `sensor_msgs/JointState` | All joint positions/velocities |

### Camera

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB camera feed |
| `/camera/color/image_compressed` | `sensor_msgs/CompressedImage` | Compressed RGB (lower bandwidth) |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth camera feed |
| `/camera/pan_tilt_cmd` | `std_msgs/Float64MultiArray` | Pan/tilt angles [pan, tilt] radians |

## Example: Python Control Script

Create a file `my_robot_control.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tidybot_msgs.msg import ArmCommand

class MyRobotController(Node):
    def __init__(self):
        super().__init__('my_robot_controller')

        # Publishers (same topics work in sim and real!)
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(ArmCommand, '/right_arm/command', 10)

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        # Move base forward
        vel = Twist()
        vel.linear.x = 0.1
        self.base_pub.publish(vel)

        # Move arm to home position
        arm_cmd = ArmCommand()
        arm_cmd.joint_positions = [0.0, -0.5, 0.5, 0.0, 0.0]
        arm_cmd.duration = 2.0
        self.arm_pub.publish(arm_cmd)

def main():
    rclpy.init()
    node = MyRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it:
```bash
# This SAME script works for simulation and real robot!
python3 my_robot_control.py
```

## Visualization with RViz

Launch RViz on your client machine to visualize the robot:

```bash
# Using the client launch file
ros2 launch tidybot_client client_rviz.launch.py

# Or manually
rviz2 -d $(ros2 pkg prefix tidybot_bringup)/share/tidybot_bringup/rviz/tidybot.rviz
```

## Troubleshooting

### No topics visible (`ros2 topic list` empty)

1. **Check ROS_DOMAIN_ID matches:**
   ```bash
   echo $ROS_DOMAIN_ID  # Should be same on robot and client
   ```

2. **Check RMW implementation matches:**
   ```bash
   echo $RMW_IMPLEMENTATION  # Should be same on both
   ```

3. **Check network connectivity:**
   ```bash
   ping <ROBOT_IP>
   ```

4. **Check firewall (temporarily disable for testing):**
   ```bash
   sudo ufw disable
   ```

5. **Try explicit peer list (Cyclone DDS):**
   Edit `cyclone_dds_client.xml` and add robot IP explicitly.

### Camera images not appearing / slow

1. **Use compressed images:**
   ```bash
   ros2 topic echo /camera/color/image_compressed
   ```

2. **Check bandwidth:**
   - Raw images: ~30 MB/s
   - Compressed images: ~1-2 MB/s

3. **Reduce resolution** in launch file parameters.

### High latency

1. **Check WiFi signal strength**
2. **Use Ethernet if possible**
3. **Reduce camera frame rate:**
   ```bash
   ros2 launch tidybot_bringup robot.launch.py camera_fps:=15
   ```

### "Failed to connect to arm"

1. **Check USB connections** on robot
2. **Check device permissions:**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # Or add to dialout group permanently
   sudo usermod -a -G dialout $USER
   ```

## Environment Variables Summary

```bash
# Add to ~/.bashrc on both robot and client

# Required
export ROS_DOMAIN_ID=42                           # Must match!
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp      # Or rmw_fastrtps_cpp

# Optional (for Cyclone DDS)
export CYCLONEDDS_URI=file:///path/to/config.xml

# Optional (for FastDDS)
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/config.xml

# Robot only
export TIDYBOT2_PATH=/home/locobot/tidybot2
```

## Security Notes

- ROS2 DDS traffic is **not encrypted by default**
- For production use, enable DDS security features
- Don't expose ROS2 ports to the internet
- Use VPN for remote access outside local network

## Further Reading

- [ROS2 Networking Concepts](https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html)
- [Cyclone DDS Configuration](https://cyclonedds.io/docs/)
- [FastDDS Discovery Server](https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html)
