# Collaborative Robotics 2026

This repository contains software for controlling the **TidyBot2** mobile robot with **WX200** 6-DOF arm, developed for Professor Monroe Kennedy's 2026 Collaborative Robotics Class.

## Quick Start

### Installation

This project uses [uv](https://docs.astral.sh/uv/) for fast Python package management.

```bash
# Install uv if you haven't already
curl -LsSf https://astral.sh/uv/install.sh | sh

# Clone the repository
git clone https://github.com/armlabstanford/collaborative-robotics-2026.git
cd collaborative-robotics-2026
```

# Install dependencies

```bash
uv sync
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-rviz2
```

### Run the Simulation

Open a terminal tab and run the following:

```bash
cd ros2_ws
colcon build 
uv sync # if .venv hasn't been generated before
source setup_env.bash
ros2 launch tidybot_bringup sim.launch.py
```

This will open Rviz2 as well as MuJoCo.

In a second terminal tab, run the following:

```bash
cd ros2_ws
source setup_env.bash
ros2 run tidybot_bringup test_base.py
```

This will run the robot in simulation

### Run on Real Robot


## Repository Structure


## Robot Specifications

**TidyBot2** is a mobile manipulation platform consisting of:
- **Mobile Base**: Kobuki or Create3 base with 3 DOF (x, y, theta)
- **WX200 Arm**: 5 DOF manipulator (550mm reach, 200g payload)
  - Waist (base rotation): ±175°
  - Shoulder (lift): -108° to 114°
  - Elbow (bend): -123° to 92°
  - Wrist angle: -100° to 123°
  - Wrist rotate: ±175°
- **Robotiq 2F-85 Gripper**: Adaptive parallel jaw gripper (85mm max opening)



## Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [uv Documentation](https://docs.astral.sh/uv/)
- [TidyBot and TidyBot2 Paper](https://arxiv.org/abs/2305.05658, https://arxiv.org/pdf/2412.10447)
- [Interbotix WX200 Specs](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx200.html)
