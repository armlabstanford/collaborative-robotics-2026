# Collaborative Robotics 2026

This repository contains software for controlling the **TidyBot2** mobile robot with bimanual **WX200** 5-DOF arms, developed for Professor Monroe Kennedy's 2026 Collaborative Robotics Class.

## Installation

There are two ways to get started:

### Option A: Docker (Any OS)

Use our pre-built Docker image with VNC desktop access. Works on any OS (Windows, macOS, Linux).

```bash
# 1. Clone the repo
git clone https://github.com/armlabstanford/collaborative-robotics-2026.git
cd collaborative-robotics-2026

# 2. Pull the image
docker pull peasant98/tidybot2:humble

# 3. Run with the repo mounted (changes sync between host and container)
docker run -p 6080:80 --shm-size=2g \
    -v $(pwd):/home/ubuntu/Desktop/collaborative \
    peasant98/tidybot2:humble

# Access via browser: http://127.0.0.1:6080/
```

**Syncing updates:** With the volume mount, any `git pull` on your host machine will immediately reflect inside the running container.

**Available commands in container:**
| Command | Description |
|---------|-------------|
| `tidybot-sim` | MuJoCo standalone simulation |
| `tidybot-ros` | ROS2 + RViz + MuJoCo |
| `tidybot-ros-no-rviz` | ROS2 + MuJoCo (no RViz) |
| `tidybot-test-base` | Test base movement |
| `tidybot-test-arms` | Test arm control |

### Option B: Native Ubuntu 22.04

If you have Ubuntu 22.04 (native install, dual-boot, or VM), use the setup script:

```bash
# Clone the repository
git clone https://github.com/armlabstanford/collaborative-robotics-2026.git
cd collaborative-robotics-2026

# Run the setup script (installs ROS2, dependencies, and builds workspace)
./setup.sh --install
```

The setup script handles everything: system dependencies, ROS2 Humble, Python environment (`uv sync`), and building the ROS2 workspace (`colcon build`).

## Quick Start

### Option 1: Standalone MuJoCo Simulation (No ROS2)

```bash
cd simulation/scripts

# Bimanual arm demo with camera control
uv run python test_move.py

# Object manipulation demo
uv run python pick_up_block.py
```

### Option 2: Full ROS2 Simulation

**Terminal 1 - Launch simulation:**
```bash
cd ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup sim.launch.py
```

This opens RViz2 and MuJoCo viewer.

**Terminal 2 - Run test scripts:**
```bash
cd ros2_ws
source setup_env.bash

# Test base movement
ros2 run tidybot_bringup test_base.py

# Test bimanual arms
ros2 run tidybot_bringup test_arms.py

# Test camera pan-tilt
ros2 run tidybot_bringup test_camera.py

# Advanced state machine example
ros2 run tidybot_bringup example_state_machine.py
```

**Launch Options:**
```bash
# Disable RViz
ros2 launch tidybot_bringup sim.launch.py use_rviz:=false

# Disable MuJoCo viewer
ros2 launch tidybot_bringup sim.launch.py show_mujoco_viewer:=false
```

## Repository Structure

```
collaborative-robotics-2026/
├── simulation/                  # Standalone MuJoCo simulation
│   ├── scripts/                 # test_move.py, pick_up_block.py
│   └── assets/                  # MuJoCo models and meshes
│
└── ros2_ws/                     # ROS2 workspace
    ├── setup_env.bash           # Environment setup script
    └── src/
        ├── tidybot_bringup/     # Launch files & test scripts
        ├── tidybot_description/ # URDF/XACRO robot model
        ├── tidybot_msgs/        # Custom ROS2 messages
        ├── tidybot_mujoco_bridge/  # MuJoCo-ROS2 bridge
        └── tidybot_control/     # Arm/base controllers
```

## Robot Specifications

**TidyBot2** is a mobile manipulation platform consisting of:
- **Mobile Base**: Kobuki or Create3 base with 3 DOF (x, y, theta)
- **Arms**: 2x WX200 5-DOF manipulators (550mm reach, 200g payload)
  - Waist (base rotation): ±175°
  - Shoulder (lift): -108° to 114°
  - Elbow (bend): -123° to 92°
  - Wrist angle: -100° to 123°
  - Wrist rotate: ±175°
- **Grippers**: 2x Robotiq 2F-85 adaptive parallel jaw (85mm max opening)
- **Camera**: Pan-tilt RealSense D435 (RGB + Depth)

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Base velocity |
| `/left_arm/command` | ArmCommand | Left arm control |
| `/right_arm/command` | ArmCommand | Right arm control |
| `/left_gripper/command` | GripperCommand | Left gripper |
| `/right_gripper/command` | GripperCommand | Right gripper |
| `/camera/pan_tilt` | PanTilt | Camera orientation |
| `/joint_states` | sensor_msgs/JointState | Joint feedback |

## Troubleshooting

**MuJoCo rendering issues:**
```bash
sudo apt install -y mesa-utils
glxinfo | grep "OpenGL version"
```

**Python import errors:**
```bash
# Always source environment first
source ros2_ws/setup_env.bash
```

**colcon build fails:**
```bash
# Ensure ROS2 is sourced before building
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build
```

## Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [uv Documentation](https://docs.astral.sh/uv/)
- [TidyBot Paper](https://arxiv.org/abs/2305.05658)
- [TidyBot2 Paper](https://arxiv.org/pdf/2412.10447)
- [Interbotix WX200 Specs](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx200.html)

## Authors

Alex Qiu & Matt Strong - Stanford ARM Lab
