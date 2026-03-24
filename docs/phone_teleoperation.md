# Phone Teleoperation System

## Overview

The phone teleoperation system enables real-time control of the TidyBot2 mobile manipulator --- including its holonomic base and dual Trossen Robotics WidowX-250 6-DOF arms --- using one or two smartphones as spatial controllers. The system leverages WebXR's immersive-ar mode to obtain full 6-DOF pose tracking (position + orientation) from the phone's built-in camera and IMU, transmitting commands to the robot over a local WiFi network via WebSocket.

The robot's arms are Trossen Robotics WidowX-250 manipulators --- the older variant commonly found on LoCoBot platforms --- mounted side by side for bimanual manipulation. Each arm has 6 degrees of freedom (waist, shoulder, elbow, forearm roll, wrist angle, wrist rotate) plus a prismatic gripper.

The design supports **bimanual teleoperation**: two phones can simultaneously control the left and right arms independently, while either phone can also drive the mobile base. This approach eliminates the need for specialized teleoperation hardware (e.g., SpaceMouse, VR headset) and requires only a commodity smartphone with AR capabilities.

## System Architecture

```
┌──────────────────┐    Socket.IO     ┌────────────────────────────┐   ROS2 Topics   ┌──────────────────┐
│  Phone 1         │   (WebSocket)    │   Phone Teleop Server      │                 │  Robot           │
│  (AR Browser)    │◄───────────────► │   Flask + ROS2 Node        │◄──────────────► │  (Sim / Real)    │
│                  │  6-DOF pose,     │                            │  /cmd_vel       │                  │
│  Controls:       │  touch events,   │  - ArmIKController         │  /*/joint_cmd   │  Publishes:      │
│  Right Arm +     │  buttons         │    (pinocchio IK)          │  /*/gripper/cmd │   /odom          │
│  Base            │                  │  - ArmTeleopController     │                 │   /*/joint_states│
└──────────────────┘                  │    (1:1 pose mapping)      │  /odom          │                  │
                                      │  - BaseTeleopController    │  /*/joint_states│  Subscribes:     │
┌──────────────────┐    Socket.IO     │    (P-control + odom)      │                 │   /cmd_vel       │
│  Phone 2         │   (WebSocket)    │  - PhoneTeleopNode         │                 │   /*/joint_cmd   │
│  (AR Browser)    │◄───────────────► │    (ROS2 pub/sub)          │                 │   /*/gripper/cmd │
│                  │                  │                            │  * = left_arm   │                  │
│  Controls:       │                  │  Port 5000 (default)       │    or right_arm │                  │
│  Left Arm +      │                  │                            │                 │                  │
│  Base            │                  │                            │                 │                  │
└──────────────────┘                  └────────────────────────────┘                 └──────────────────┘
```

### Communication Layers

The system employs two distinct communication layers:

1. **Phone-to-Server (Socket.IO over WiFi):** Each phone runs a WebXR AR session in the browser. Every animation frame (~60 Hz from WebXR, throttled to 20 Hz by the server's control loop), the phone emits a Socket.IO message containing its 6-DOF pose, the active teleop mode (`arm` or `base`), and touch input data. The server echoes back timestamps for round-trip time (RTT) measurement. Typical RTT on 5 GHz WiFi is ~7 ms.

2. **Server-to-Robot (ROS2 topics):** The server node publishes velocity commands (`/cmd_vel` for the base) and joint position commands (`/right_arm/joint_cmd`, `/left_arm/joint_cmd`) at a fixed 20 Hz control rate. It subscribes to joint state feedback (`/right_arm/joint_states`, `/left_arm/joint_states`) for real-time IK seeding, and to odometry (`/odom`) for closed-loop base position control.

### Threading Model

The server runs three concurrent threads:

- **Flask/Socket.IO thread:** Runs the Werkzeug HTTP server on port 5000. Handles WebSocket connections, serves the HTML/JS interface, and routes incoming messages into per-device queues.
- **ROS2 spin thread:** Processes ROS2 callbacks (joint state and odometry subscriptions) in a separate daemon thread.
- **Control loop (ROS2 timer):** A 20 Hz timer callback on the main ROS2 node that drains per-device message queues, computes IK solutions and base velocity commands, applies velocity limiting, and publishes to ROS2 topics.

### Message Flow

```
Phone (WebXR frame, ~60 Hz)
  │
  ▼
Socket.IO 'message' event
  │
  ▼
Per-device Queue (bounded, size 20, drops oldest on overflow)
  │
  ▼
Control Loop (20 Hz timer)
  ├─ Drain queue → keep latest non-stale message (< 250 ms old)
  ├─ TeleopController.process_message()
  │   ├─ Base mode → position displacement + P controller → cmd_vel
  │   └─ Arm mode  → 1:1 pose mapping + IK solver → target joint positions
  ├─ Velocity-limited interpolation (0.5 rad/s max per joint, or 1/6 in slow mode)
  └─ Publish to ROS2 topics
```

## Network Connection

### Requirements

- The phone and the computer running the teleop server must be on the **same WiFi network**.
- **5 GHz WiFi** is strongly recommended for low latency (~7 ms RTT). 2.4 GHz WiFi will work but with higher and more variable latency.
- The server binds to `0.0.0.0:5000` by default, accepting connections from any device on the network.
- The phone connects by navigating to `http://<SERVER_IP>:5000` in an AR-capable browser.

### Phone Browser Requirements

WebXR's `immersive-ar` session is required, which is **not** supported by all browsers:

| Platform | Supported Browsers | Notes |
|----------|-------------------|-------|
| **iPhone** | XRViewer, XR Browser | Safari does **not** support WebXR. Install from the App Store. |
| **Android** | Chrome | Requires an ARCore-supported device. |

### How to Connect

1. Start the robot (simulation or real hardware) and the phone teleop server (see [Launching](#launching) below).
2. Find the server machine's IP address: `hostname -I`
3. On the phone, open the AR browser and navigate to `http://<SERVER_IP>:5000`
4. The phone displays the TidyBot2 teleop UI with a WebXR button.
5. Tap **"Start episode"** to begin the WebXR AR session. The camera feed appears with a semi-transparent overlay.
6. For bimanual control, repeat steps 2--5 on a second phone. The first phone is auto-assigned to the right arm; the second to the left arm.

## User Interface

### Screen Layout

When the WebXR session is active, the phone displays the live camera feed with a DOM overlay containing:

```
┌─────────────────────────────────────────────────────────────┐
│ [Start episode / End episode / Reset env]                   │  ← WebXR button (top)
├─────────────────────────────────────────────────────────────┤
│ My arm: RIGHT  [Left] [Right] [Gripper: OPEN]               │  ← Arm controls bar
│                [Slow: OFF] [Home] [Sleep] [STOP]            │
├─────────────────────────────────────────────────────────────┤
│ RTT: 5.2/6.8/8.1 ms                                         │  ← Network stats
├──────────────────────────────────────────────┬──────────────┤
│                                              │              │
│           Arm Control Zone                   │  Base        │
│           (left 90% of screen)               │  Control     │
│                                              │  Zone        │
│  Touch + move phone = arm EE follows 1:1     │  (right 10%) │
│  Touch + rotate phone = EE rotation 1:1      │              │
│  Vertical swipe = gripper open/close         │  Touch +     │
│                                              │  move phone  │
│                                              │  = base moves│
│                                              │  1:1         │
└──────────────────────────────────────────────┴──────────────┘
```

### On-Screen Buttons

| Button | Function |
|--------|----------|
| **Start episode** | Begin WebXR AR session and teleoperation. Button text cycles through three states. |
| **End episode** | Signal that the current teleoperation episode has ended (for data collection workflows). |
| **Reset env** | End the AR session entirely and reset. |
| **Left / Right** | Manually assign this phone to control the left or right arm. The active arm is highlighted in blue. |
| **Gripper: OPEN / CLOSED** | Toggle the gripper of the assigned arm. Green = open, red = closed. |
| **Slow: OFF / ON** | Toggle slow mode, which reduces arm joint velocity to 1/3 of normal for fine manipulation. Purple = off, orange = on. |
| **Home** | Send the assigned arm to the home pose `[0.0, 0.0, 0.0, 0.0, 1.57, 0.0]` rad (wrist angle at 90 deg). |
| **Sleep** | Send the assigned arm to the sleep/tucked pose `[0.0, -1.80, 1.55, 0.0, 0.8, 0.0]` rad. |
| **STOP** | Emergency stop --- halts all motion, zeroes base velocity, clears all device assignments. |

### Visual Feedback

The AR camera overlay changes color to indicate the active control mode:

| Overlay Color | Meaning |
|---------------|---------|
| **Red tint** (r=0.25) | Base control is active (touching right 10% of screen) |
| **Blue tint** (b=0.25--1.0) | Arm control is active (touching left 90% of screen). Intensity scales with vertical swipe for gripper feedback. |
| **No tint** (transparent) | Idle --- not touching the screen |

### RTT Display

The UI continuously displays min/avg/max round-trip time over a sliding window of the last 100 measurements. This helps the operator assess network quality in real time.

## Control Modes

### Touchscreen Region Mapping

The phone screen is divided into two touch regions:

- **Left 90%**: Arm control mode. Touching this region and moving/rotating the phone controls the end-effector of the assigned arm.
- **Right 10%**: Base control mode. Touching this region and moving/rotating the phone drives the mobile base.

The region is determined at `touchstart` based on the X coordinate of the initial touch: `t.clientX < 0.9 * window.innerWidth` selects arm mode, otherwise base mode. The mode persists until `touchend`.

### Base Control (Position-Displacement with P Controller)

When the operator touches the right 10% of the screen, the phone acts as a **physical proxy for the robot base**. Physically moving and rotating the phone in space commands the robot to move by the same displacement.

**Reference frame capture:** On the first touch frame, the system captures:
- The phone's current position and orientation as a reference pose
- The robot's current position and heading from odometry (`/odom`)

**Target computation:** As the operator moves the phone, the displacement from the reference phone pose is added to the reference robot pose to produce a target:

$$
\mathbf{p}_{\text{target}} = \mathbf{p}_{\text{robot\_ref}} + (\mathbf{p}_{\text{phone}} - \mathbf{p}_{\text{phone\_ref}})
$$

$$
\theta_{\text{target}} = \theta_{\text{robot\_ref}} + \Delta\theta_{\text{phone\_yaw}}
$$

The mapping is 1:1 in world frame coordinates --- the phone's WebXR session frame and the robot's odometry frame share the same orientation, so no rotation transform is needed.

**Closed-loop P controller:** Rather than sending the target pose directly, a proportional controller computes velocity commands that drive the robot toward the target using odometry feedback:

1. **Position error** is computed in the odometry frame and then rotated into the robot's body frame:

$$
\begin{bmatrix} e_x^{\text{body}} \\ e_y^{\text{body}} \end{bmatrix} = \begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} x_{\text{target}} - x_{\text{odom}} \\ y_{\text{target}} - y_{\text{odom}} \end{bmatrix}
$$

2. **Heading error** is wrapped to $[-\pi, \pi]$:

$$
e_\theta = \big((\theta_{\text{target}} - \theta_{\text{odom}}) + \pi\big) \bmod 2\pi - \pi
$$

3. **Velocity commands** are computed with proportional gains and clamped:

$$
v_x = \text{clamp}(K_p^{\text{pos}} \cdot e_x^{\text{body}},\; \pm v_{\text{max}})
$$

$$
v_y = \text{clamp}(K_p^{\text{pos}} \cdot e_y^{\text{body}},\; \pm v_{\text{max}})
$$

$$
\omega_z = \text{clamp}(K_p^{\text{heading}} \cdot e_\theta,\; \pm \omega_{\text{max}})
$$

| Parameter | Value | Description |
|-----------|-------|-------------|
| Position P gain ($K_p^{\text{pos}}$) | 2.0 | Proportional gain for XY position |
| Heading P gain ($K_p^{\text{heading}}$) | 3.0 | Proportional gain for heading |
| Max linear velocity ($v_{\text{max}}$) | 0.5 m/s | Caps translational speed |
| Max angular velocity ($\omega_{\text{max}}$) | 1.0 rad/s | Caps rotational speed |
| Position deadzone | 0.01 m | Ignores position errors smaller than this |
| Heading deadzone | 0.03 rad (~2 deg) | Ignores heading errors smaller than this |

**Stopping:** When the operator releases the touch, the base velocity is immediately set to zero.

**Advantages of closed-loop position control:** Unlike open-loop velocity mapping (e.g., tilt-based), this approach compensates for disturbances and drift --- the P controller continuously corrects toward the commanded target using odometry feedback. The 1:1 displacement mapping also provides an intuitive physical correspondence: moving the phone 10 cm forward moves the robot 10 cm forward.

### Arm Control (Direct 1:1 Phone-to-EE Mapping via IK)

When the operator touches the left 90% of the screen, the phone acts as a **physical proxy for the robot's end-effector (gripper)**. Moving and rotating the phone maps directly to matching EE displacement and rotation, solved via real-time inverse kinematics.

#### Position Control

On the first touch frame, the phone's position and the arm's current EE position (computed via forward kinematics) are both captured as references. Subsequent phone displacement is mapped 1:1 to EE displacement in all three axes:

$$
\mathbf{p}_{\text{target\_ee}} = \mathbf{p}_{\text{ref\_ee}} + (\mathbf{p}_{\text{phone}} - \mathbf{p}_{\text{ref\_phone}})
$$

There is no scaling factor --- phone displacement maps directly to EE displacement. This provides an intuitive physical correspondence where the phone acts as a stand-in for the gripper.

#### Orientation Control

Phone rotation is mapped directly to EE rotation via relative rotation from the reference:

$$
R_{\text{target\_ee}} = (\Delta R_{\text{phone}}) \cdot R_{\text{ref\_ee}}
$$

where $\Delta R_{\text{phone}} = R_{\text{phone}} \cdot R_{\text{ref\_phone}}^{-1}$ is the phone's rotation since touch-start. This gives 1:1 correspondence --- rotating the phone 30 degrees rotates the EE 30 degrees about the same axis.

#### Gripper Control (Vertical Swipe)

While in arm control mode, the vertical swipe distance on the touchscreen controls the gripper:

$$
\text{gripperDelta} = \frac{y_{\text{start}} - y_{\text{current}}}{0.2 \cdot h_{\text{window}}} \quad \in [-1, 1]
$$

| Swipe | Threshold | Action |
|-------|-----------|--------|
| Swipe up (finger moves up) | gripperDelta < -0.3 | Close gripper (1.0) |
| Swipe down (finger moves down) | gripperDelta > 0.3 | Open gripper (0.0) |
| Small swipe | |gripperDelta| <= 0.3 | No change |

The gripper command is binary (fully open or fully closed), toggled by crossing the threshold.

### Coordinate System Conversion

WebXR and the robot use different coordinate conventions. All poses received from WebXR are converted before processing:

```
WebXR:  +x right,    +y up,      +z back     (right-handed)
Robot:  +x forward,   +y left,    +z up       (right-handed, ROS convention)
```

**Position conversion:**
$$
\mathbf{p}_{\text{robot}} = [-z_{\text{xr}},\; -x_{\text{xr}},\; y_{\text{xr}}]
$$

**Orientation conversion (quaternion):**
$$
\mathbf{q}_{\text{robot}} = [-z_{\text{xr}},\; -x_{\text{xr}},\; y_{\text{xr}},\; w_{\text{xr}}]
$$

**Device camera offset:** A correction of `[0.0, 0.02, -0.04]` meters is applied (in the phone's rotated frame) so that rotations are computed around the device's physical center rather than the camera lens position.

## Inverse Kinematics

### Pinocchio-Based IK Solver with Nullspace Optimization

The arm IK uses the Pinocchio rigid-body dynamics library with a damped least-squares (Levenberg-Marquardt) approach, augmented with **nullspace optimization** that biases redundant degrees of freedom toward a comfortable rest pose.

The URDF is loaded from the `tidybot_description` package and processed via `xacro`.

#### Algorithm

Given a target EE position $\mathbf{p}^*$ and orientation $R^*$, and a seed configuration $\mathbf{q}_0$:

```
for iteration = 1 to max_iter:
    Compute FK: current EE pose (p_current, R_current) from q
    Position error:    e_pos = p* - p_current
    Orientation error: e_ori = log3(R*^T @ R_current)

    if ||e_pos|| < pos_tol and ||e_ori|| < ori_tol:
        break  (converged)

    Stack error vector: e = [e_pos; -e_ori]  (6x1)

    Compute numerical Jacobian J (6x6) by finite differences (eps = 1e-4)

    Compute damped pseudo-inverse: J† = J^T @ (J @ J^T + lambda * I)^{-1}

    Primary task velocity:     v_primary = J† @ e
    Nullspace projector:       N = I - J† @ J
    Nullspace bias:            v_null = alpha * N @ (q_rest - q_current)
    Combined step:             v = v_primary + v_null

    Clamp max step: if max(|v|) > max_step, scale v down proportionally

    Update: q[i] += dt * v[i],  clamped to joint limits
```

The nullspace optimization projects a bias toward the rest pose into the null space of the task Jacobian, meaning it only influences joint motion that does not affect the EE pose. This prevents the arm from drifting into singularities or uncomfortable configurations during extended teleoperation.

#### IK Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Max iterations | 30 (per control frame) | Limits computation time |
| Step size ($dt$) | 0.3 | Controls convergence speed vs stability |
| Damping ($\lambda$) | $10^{-5}$ | Regularization for singularity robustness |
| Position tolerance | 0.01 m | IK convergence threshold for position |
| Orientation tolerance | 0.1 rad | IK convergence threshold for orientation |
| Position verification tolerance | 0.05 m | Reject solution if FK check exceeds this |
| Max step per iteration | 45 deg (0.785 rad) | Clamps joint angle change per iteration |
| Nullspace gain ($\alpha$) | 0.15 | Gentle bias toward rest pose |
| Rest pose | [0.0, 0.05, 0.5, 0.0, 0.0, 0.0] rad | Nullspace target (shoulder-neutral) |
| Finite difference epsilon | $10^{-4}$ rad | For numerical Jacobian computation |

#### Unreachable Target Handling

After IK convergence, a forward kinematics check verifies that the solved EE position is within 0.05 m of the target. If the target is unreachable, the solution is **rejected** (not sent to the robot), and the arm holds its last valid commanded position.

### Velocity-Limited Interpolation

To prevent jerky motion, IK-solved joint targets are not sent directly. Instead, the commanded positions are interpolated toward the target at each control frame with a per-joint velocity limit:

$$
\Delta q_i = \text{clamp}(q_i^{\text{target}} - q_i^{\text{current}},\; -v_{\text{max}} \cdot dt,\; +v_{\text{max}} \cdot dt)
$$

$$
q_i^{\text{cmd}} = \text{clamp}(q_i^{\text{current}} + \Delta q_i,\; q_i^{\text{min}},\; q_i^{\text{max}})
$$

where $v_{\text{max}} = 0.5$ rad/s in normal mode or $v_{\text{max}} = 0.167$ rad/s in slow mode, and $dt = 0.05$ s (20 Hz).

| Mode | Max Joint Velocity | Max Step per Frame |
|------|-------------------|-------------------|
| Normal | 0.5 rad/s | 0.025 rad |
| Slow | 0.167 rad/s (1/3) | 0.0083 rad |

## Dual-Phone Bimanual Control

### Device Auto-Assignment

Each phone generates a random 15-character device ID on page load. The server tracks device-to-arm assignments in a dictionary `{device_id: 'right' | 'left'}`.

**Auto-assignment logic:**
1. A device must send at least 2 consecutive frames with an active touch (warmup) before being assigned.
2. The **first** device to touch is assigned to the **right** arm.
3. The **second** device is assigned to the **left** arm.
4. If both arms are already assigned, additional devices are not assigned arm control.

**Manual override:** The operator can tap the "Left" or "Right" button on the phone UI to reassign. If the target arm is held by another device, the two devices swap arms.

### Independent Control

Each phone independently controls its assigned arm's EE position, orientation, and gripper. Both phones can also control the base --- when both touch the base region simultaneously, the most recent message wins.

### Disconnect Cleanup

When a phone's WebSocket disconnects, all state for that device is cleaned up: its arm assignment is removed, the arm state is reset, and if it was the active base controller, the base velocity is zeroed.

## ROS2 Interface

### Published Topics

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 20 Hz | Base velocity: `linear.x` (forward), `linear.y` (strafe), `angular.z` (rotate). Computed by P controller from position error. Zeroed when no touch. |
| `/right_arm/joint_cmd` | `std_msgs/Float64MultiArray` | 20 Hz | Right arm target joint positions (6 floats, radians). Only published when arm control is active. |
| `/left_arm/joint_cmd` | `std_msgs/Float64MultiArray` | 20 Hz | Left arm target joint positions (6 floats, radians). Only published when arm control is active. |
| `/right_gripper/cmd` | `std_msgs/Float64MultiArray` | On change | Right gripper command: `data[0]` = 0.0 (open) or 1.0 (closed). |
| `/left_gripper/cmd` | `std_msgs/Float64MultiArray` | On change | Left gripper command: `data[0]` = 0.0 (open) or 1.0 (closed). |
| `/camera/pan_tilt_cmd` | `std_msgs/Float64MultiArray` | --- | Camera pan-tilt (reserved, not actively used). |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/odom` | `nav_msgs/Odometry` | Current base pose (x, y, theta). Used by P controller for closed-loop base control. |
| `/right_arm/joint_states` | `sensor_msgs/JointState` | Current right arm joint positions (first 6 values). Used as IK seed. |
| `/left_arm/joint_states` | `sensor_msgs/JointState` | Current left arm joint positions (first 6 values). Used as IK seed. |

### Socket.IO Events

| Event | Direction | Data | Purpose |
|-------|-----------|------|---------|
| `message` | Phone -> Server | `{timestamp, device_id, teleop_mode, position, orientation, gripper_delta}` | Continuous teleoperation pose data (every WebXR frame) |
| `echo` | Server -> Phone | `timestamp` | Echo back for RTT calculation |
| `set_arm` | Phone -> Server | `{device_id, arm}` | Manually assign this phone to 'right' or 'left' arm |
| `request_assignment` | Phone -> Server | `{device_id}` | Query current arm assignment (sent on reconnect) |
| `arm_assignment` | Server -> Phone | `{device_id, arm, all_assignments}` | Broadcast current arm assignments to all clients |
| `toggle_gripper` | Phone -> Server | `{device_id, side}` | Toggle the gripper of the specified arm |
| `preset_pose` | Phone -> Server | `{device_id, arm, pose}` | Send arm to 'home' or 'sleep' preset |
| `set_slow_mode` | Phone -> Server | `{enabled}` | Toggle slow mode (1/3 velocity) |
| `emergency_stop` | Phone -> Server | (none) | Stop all motion immediately |

## WidowX-250 Arm Joint Limits

| Joint | Index | Min (rad) | Max (rad) | Range (deg) |
|-------|-------|-----------|-----------|-------------|
| Waist | 0 | -3.14 | 3.14 | 360 |
| Shoulder | 1 | -1.88 | 1.99 | 222 |
| Elbow | 2 | -2.15 | 1.61 | 215 |
| Forearm Roll | 3 | -3.14 | 3.14 | 360 |
| Wrist Angle | 4 | -1.75 | 2.15 | 223 |
| Wrist Rotate | 5 | -3.14 | 3.14 | 360 |

**Preset poses (radians):**

| Pose | Waist | Shoulder | Elbow | Forearm Roll | Wrist Angle | Wrist Rotate |
|------|-------|----------|-------|-------------|-------------|--------------|
| Home | 0.0 | 0.0 | 0.0 | 0.0 | 1.57 | 0.0 |
| Sleep | 0.0 | -1.80 | 1.55 | 0.0 | 0.8 | 0.0 |

## Launching

### Prerequisites

```bash
# Install web dependencies (flask, flask-socketio, scipy)
cd ~/collaborative-robotics-2026
uv sync --extra web

# Build ROS2 workspace
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tidybot_control tidybot_bringup
source install/setup.bash
```

### Start the System

**Terminal 1 --- Robot (simulation or real):**
```bash
cd ~/collaborative-robotics-2026/ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup sim.launch.py       # simulation
# or: ros2 launch tidybot_bringup real.launch.py  # real hardware
```

**Terminal 2 --- Phone teleop server:**
```bash
cd ~/collaborative-robotics-2026/ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup phone_teleop.launch.py
```

Or run the node directly:
```bash
ros2 run tidybot_control phone_teleop_server
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `port` | 5000 | HTTP/WebSocket server port |
| `host` | 0.0.0.0 | Network interface to bind to |

```bash
ros2 launch tidybot_bringup phone_teleop.launch.py port:=8080 host:=192.168.1.100
```

## Implementation Details

### Modular Architecture

The teleop system is split across three Python files:

| File | Lines | Purpose |
|------|-------|---------|
| `phone_teleop_server.py` | ~900 | Main server: Flask app, Socket.IO handlers, ROS2 node, embedded HTML/JS UI, WebXR button component, control loop orchestration |
| `base_teleop.py` | ~130 | Base controller: position-displacement mapping, P controller, odom integration |
| `arm_teleop.py` | ~390 | Arm controller: pinocchio FK/IK with nullspace optimization, per-arm state management, device-to-arm assignment |

The `TeleopController` class in `phone_teleop_server.py` composes `BaseTeleopController` and `ArmTeleopController`, delegating base and arm messages to the appropriate sub-controller.

The phone UI (HTML, CSS, JavaScript) and the WebXR button component are embedded as Python string templates within `phone_teleop_server.py`, served via Flask's `render_template_string`. No separate static files are needed.

### Dependencies

| Library | Purpose |
|---------|---------|
| Flask + Flask-SocketIO | HTTP server + WebSocket transport |
| Pinocchio | URDF parsing, FK, and numerical Jacobian for IK |
| SciPy (Rotation) | Quaternion/Euler/rotation-vector conversions |
| NumPy | Vectorized pose math |
| rclpy | ROS2 Python client |

**Client-side (loaded via CDN):**

| Library | Purpose |
|---------|---------|
| Socket.IO 4.7.5 | WebSocket client |
| WebXR Device API | 6-DOF pose tracking via phone AR |

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| "AR NOT FOUND" button | Browser lacks WebXR immersive-ar support | Use XRViewer/XR Browser on iPhone, Chrome on Android |
| Phone can't connect | Not on same WiFi, or firewall blocking port | Verify with `ping`; run `sudo ufw allow 5000/tcp` |
| High RTT (>20 ms) | 2.4 GHz WiFi, interference, distance | Switch to 5 GHz WiFi, move closer to AP |
| Arms not moving | Joint states not being published | Verify: `ros2 topic echo /right_arm/joint_states` |
| Base not moving | Odom not published or wrong touch region | Verify: `ros2 topic echo /odom`; touch the rightmost 10% of the screen |
| Base drifts | Odom inaccurate or not published | Check odometry source; P controller requires odom feedback |
| IK not converging | Target far from current pose | Move phone slowly; arm will hold last valid position at workspace boundaries |

## Differences from Upstream tidybot2

This implementation adapts the phone teleoperation scheme from the [upstream tidybot2 project](https://github.com/jimmyyhwu/tidybot2) with the following changes:

| Aspect | Upstream tidybot2 | This Implementation |
|--------|-------------------|---------------------|
| Robot arms | Kinova Gen3 (7-DOF) with IK + torque control | WidowX-250 (6-DOF) with pinocchio IK + position control |
| Base control | Position-displacement via custom RPC | Position-displacement with P controller via ROS2 `/cmd_vel` + `/odom` |
| Arm control interface | Custom RPC to arm_server.py | ROS2 `/right_arm/joint_cmd` + `/left_arm/joint_cmd` |
| Arm IK | Proprietary solver | Pinocchio damped least-squares with nullspace optimization |
| Communication (robot side) | Custom RPC servers | Standard ROS2 pub/sub |
| Episode recording | Built-in (episode_storage.py) | Not included |
| Policy server | Diffusion policy execution via ZMQ | Not included |
| WebXR + touch regions | 90/10 split, same control scheme | Same 90/10 split |
| Dual-phone bimanual | Supported | Supported (with auto-assignment) |

## Control Parameters Summary

Tunable parameters are defined at the top of each module:

### Base Control (`base_teleop.py`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `POS_P_GAIN` | 2.0 | Proportional gain for XY position |
| `HEADING_P_GAIN` | 3.0 | Proportional gain for heading |
| `BASE_MAX_LINEAR_VEL` | 0.5 m/s | Maximum base translational speed |
| `BASE_MAX_ANGULAR_VEL` | 1.0 rad/s | Maximum base rotational speed |
| `POS_DEADZONE` | 0.01 m | Ignore position errors smaller than this |
| `HEADING_DEADZONE` | 0.03 rad (~2 deg) | Ignore heading errors smaller than this |

### Arm Control (`arm_teleop.py`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `ARM_IK_POS_TOL` | 0.05 m | Reject IK solution if position error exceeds this |
| `ARM_IK_MAX_STEP` | 45 deg | Max joint angle change per IK iteration |
| `ARM_IK_NULLSPACE_GAIN` | 0.15 | Bias strength toward rest pose |
| `REST_POSE` | [0.0, 0.05, 0.5, 0.0, 0.0, 0.0] rad | Nullspace optimization target |

### Server (`phone_teleop_server.py`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `CONTROL_RATE` | 20 Hz | Server control loop frequency |
| `MAX_JOINT_VELOCITY` | 0.5 rad/s | Per-joint velocity limit for smooth motion |
| `MESSAGE_TIMEOUT_MS` | 250 ms | Discard stale WebXR messages older than this |
| `DEVICE_CAMERA_OFFSET` | [0.0, 0.02, -0.04] m | iPhone camera-to-center offset correction |
