# Phone Teleoperation Setup Guide

Control TidyBot2's mobile base and bimanual arms from your phone using WebXR. This uses the same control scheme as the [upstream tidybot2 project](https://github.com/jimmyyhwu/tidybot2) — the phone acts as a 6-DOF spatial controller via AR camera tracking.

## Architecture

```
┌──────────────────┐  Socket.IO (WiFi)  ┌─────────────────────┐  ROS2 Topics  ┌───────────┐
│  Phone           │ ◄────────────────► │  Phone Teleop Server │ ◄──────────► │  Robot     │
│  (AR Browser)    │  WebXR 6-DOF pose   │  (Flask + ROS2 Node) │  target_pose │  (Sim/Real)│
│  XRViewer/Chrome │                     │  Port 5000           │  joint_cmd   │            │
└──────────────────┘                     └─────────────────────┘              └───────────┘
```

The phone runs a **WebXR immersive-ar session**, which provides full 6-DOF pose tracking (position + orientation) via the phone's camera. Phone movement is translated to robot base and arm commands.

## Prerequisites

### 1. Install Web Dependencies

```bash
cd ~/collaborative-robotics-2026
uv sync --extra web
```

This installs `flask`, `flask-socketio`, and `scipy` (for rotation math).

### 2. Build the ROS2 Workspace

```bash
cd ~/collaborative-robotics-2026/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tidybot_control tidybot_bringup
source install/setup.bash
```

### 3. Phone Requirements

WebXR requires an **AR-capable browser**:

| Platform | Browser | Notes |
|----------|---------|-------|
| **iPhone** | [XRViewer](https://apps.apple.com/app/xr-browser/id1588029989) or [XR Browser](https://apps.apple.com/app/xr-browser/id1588029989) | Safari does NOT support WebXR |
| **Android** | Chrome | Requires ARCore-supported device |

### 4. Network Setup

- Phone and robot must be on the **same WiFi network**
- Use **5 GHz WiFi** for best latency (~7ms RTT expected)
- Find the robot's IP: `hostname -I`
- Ensure port 5000 is open: `sudo ufw allow 5000/tcp`

## Quick Start

### Step 1: Launch the Robot

**Simulation:**
```bash
cd ~/collaborative-robotics-2026/ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup sim.launch.py
```

**Real hardware:**
```bash
cd ~/collaborative-robotics-2026/ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup real.launch.py
```

### Step 2: Launch Phone Teleop Server

In a new terminal:
```bash
cd ~/collaborative-robotics-2026/ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup phone_teleop.launch.py
```

Or run directly:
```bash
ros2 run tidybot_control phone_teleop_server
```

### Step 3: Connect Your Phone

1. Open **XRViewer** (iPhone) or **Chrome** (Android)
2. Navigate to `http://192.168.0.208:5000`
3. Tap **"Start episode"** to begin the WebXR AR session
4. The camera view will appear with a semi-transparent overlay

## Control Scheme

The control scheme matches the [upstream tidybot2](https://tidybot2.github.io/docs/usage/#phone-teleoperation):

### Screen Regions
```
┌────────────────────────────────┬──────┐
│                                │      │
│      (left 90%)                │ CTRL │
│                                │ (10%)│
│  Touch + move phone =          │      │
│  arm joint control             │      │
│                                │      │
│  Swipe up/down =               │      │
│  gripper open/close            │      │
└────────────────────────────────┴──────┘
```

### Base Control (Touch Right 10% of Screen)

The phone acts as a **position controller** — physically moving the phone moves the robot:

| Phone Action | Robot Action |
|-------------|-------------|
| Move phone forward/back | Robot drives forward/back |
| Move phone left/right | Robot strafes left/right |
| Rotate phone (yaw) | Robot rotates in place |
| Release touch | Robot holds position |

**How it works:**
1. On first touch, captures a reference frame (phone pose + robot pose)
2. As you physically move the phone in space, the delta is applied to the robot
3. Phone XY translation maps 1:1 to robot XY translation
4. Phone yaw rotation maps to robot theta rotation

### Arm Control (Touch Left 90% of Screen)

Phone rotation maps to arm joint deltas (adapted for WX250s 6-DOF):

| Phone Motion | Robot Joint |
|-------------|-------------|
| Tilt forward/back (pitch) | Wrist Angle |
| Tilt left/right (roll) | Forearm Roll |
| Rotate phone (yaw) | Waist |
| Swipe up on screen | Open gripper |
| Swipe down on screen | Close gripper |

### Visual Feedback

The AR camera overlay changes color based on mode:
- **Red tint** = Base control active
- **Blue tint** = Arm control active (intensity indicates gripper position)
- **No tint** = Not touching (idle)

### On-Screen Controls

The DOM overlay provides additional buttons:
- **Right / Left** — Select which arm to control
- **Gripper: OPEN/CLOSED** — Toggle gripper
- **Home / Sleep** — Send arm to preset pose
- **STOP** — Emergency stop all movement

### Episode Lifecycle

The WebXR button cycles through states:
1. **"Start episode"** — Begin AR session and teleoperation
2. **"End episode"** — Signal episode end (for data collection)
3. **"Reset env"** — End session and reset

## Launch Options

```bash
# Custom port
ros2 launch tidybot_bringup phone_teleop.launch.py port:=8080

# Bind to specific interface
ros2 launch tidybot_bringup phone_teleop.launch.py host:=192.168.1.100
```

## ROS2 Topics

The server **publishes** to:

| Topic | Type | Description |
|-------|------|-------------|
| `/base/target_pose` | `geometry_msgs/Pose2D` | Base position target [x, y, theta] |
| `/cmd_vel` | `geometry_msgs/Twist` | Base velocity (emergency stop only) |
| `/right_arm/joint_cmd` | `Float64MultiArray` | Right arm 6 joint positions |
| `/left_arm/joint_cmd` | `Float64MultiArray` | Left arm 6 joint positions |
| `/right_gripper/cmd` | `Float64MultiArray` | Right gripper (0=open, 1=closed) |
| `/left_gripper/cmd` | `Float64MultiArray` | Left gripper (0=open, 1=closed) |

It **subscribes** to:

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Current base pose (for reference frame) |
| `/right_arm/joint_states` | `JointState` | Current right arm positions |
| `/left_arm/joint_states` | `JointState` | Current left arm positions |

**Note:** Base control uses **pose targets** (`/base/target_pose`), not velocity. The robot's base controller handles driving to the target position. This matches the upstream design.

## Coordinate System

WebXR and the robot use different coordinate conventions:

```
WebXR:  +x right,   +y up,   +z back
Robot:  +x forward,  +y left, +z up

Conversion: robot_pos = [-webxr_z, -webxr_x, webxr_y]
```

An iPhone camera offset (0, 0.02, -0.04) is applied so rotations are around the device center rather than the camera lens.

## Troubleshooting

### "AR NOT FOUND" button
- Your browser doesn't support WebXR immersive-ar
- **iPhone:** Use XRViewer or XR Browser (Safari doesn't support WebXR)
- **Android:** Use Chrome on an ARCore-supported device
- **Desktop:** WebXR AR is not available — this requires a phone

### Phone can't connect
- Verify same network: `ping <ROBOT_IP>`
- Check firewall: `sudo ufw allow 5000/tcp`
- Use `http://` (not `https://`)

### Base doesn't move
- Ensure `/odom` is being published: `ros2 topic echo /odom`
- The base controller must be running to accept `/base/target_pose`
- Check RTT stats on the phone — high latency means unreliable WiFi

### High latency (RTT > 20ms)
- Switch to 5 GHz WiFi
- Reduce distance to access point
- Consider USB-C Ethernet adapter for crowded environments

### Arms not moving
- Ensure joint states are published: `ros2 topic echo /right_arm/joint_states`
- Arm control uses incremental deltas from current position

## Differences from Upstream

| Aspect | Upstream tidybot2 | This Implementation |
|--------|-------------------|---------------------|
| Arms | Kinova Gen3 (7-DOF), IK + torque control | WX250s (6-DOF), joint-space incremental |
| Base interface | Custom RPC to base_server.py | ROS2 `/base/target_pose` topic |
| Arm interface | Custom RPC to arm_server.py | ROS2 `/right_arm/joint_cmd` topic |
| WebXR + base control | Same (6-DOF pose tracking) | Same (6-DOF pose tracking) |
| Touch regions | Same (90/10 split) | Same (90/10 split) |
| Episode recording | Yes (episode_storage.py) | Not included |
| Policy server | Yes (diffusion policy via ZMQ) | Not included |

## Control Parameters

Adjustable in [phone_teleop_server.py](../ros2_ws/src/tidybot_control/tidybot_control/phone_teleop_server.py):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ARM_JOINT_SPEED` | 0.8 rad/s | Max arm joint velocity from phone input |
| `CONTROL_RATE` | 20 Hz | Command publish rate |
| `MESSAGE_TIMEOUT_MS` | 250 ms | Ignore stale WebXR messages older than this |
| `DEVICE_CAMERA_OFFSET` | [0, 0.02, -0.04] | iPhone camera offset (adjust for other devices) |
