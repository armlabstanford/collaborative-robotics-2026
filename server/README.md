# Dense Mapping with Phone Teleop

## Prerequisites
- Simulation running: `ros2 launch tidybot_bringup sim.launch.py`
- GPU server deps installed: `pip install -r ~/requirements.txt` on `giuse@100.77.113.90`

## Run (3 terminals)

### Terminal 1 — ZMQ Bridge (robot)
```bash
cd ~/collaborative-robotics-2026/ros2_ws
source setup_env.bash
ros2 run tidybot_control zmq_bridge_node
```

### Terminal 2 — Phone Teleop (robot)
```bash
cd ~/collaborative-robotics-2026/ros2_ws
source setup_env.bash
ros2 run tidybot_control phone_teleop_server
```
Open `http://<ROBOT_IP>:5000` on your phone to control the robot.

### Terminal 3 — Mapping Server (GPU server)
```bash
ssh giuse@100.77.113.90
PYTHONUNBUFFERED=1 python3 mapping_server.py --robot-ip 100.106.67.118 --device CPU:0
```
Open `http://100.77.113.90:8080` in a browser to see the live 3D map.

## Options

```bash
# Use CUDA GPU for TSDF (if available)
python3 mapping_server.py --robot-ip 100.106.67.118 --device CUDA:0

# Enable ICP drift correction
python3 mapping_server.py --robot-ip 100.106.67.118 --use-icp

# Change voxel size (smaller = more detail, more memory)
python3 mapping_server.py --robot-ip 100.106.67.118 --voxel-size 0.01

# Change viser web UI port
python3 mapping_server.py --robot-ip 100.106.67.118 --vis-port 9090
```

## Data Flow

```
Phone (WebXR) → phone_teleop_server → /cmd_vel
    → MuJoCo bridge moves base, publishes /odom + TF (odom → base_link)
    → zmq_bridge_node reads /camera/* + TF (camera_color_optical_frame → odom)
    → ZMQ PUB tcp://*:5555 (JPEG RGB + lz4 depth + 4x4 pose)
    → mapping_server receives, integrates into TSDF
    → viser serves live 3D mesh at :8080
```

## Viser GUI Controls
- **Save Map** — saves mesh.ply, pointcloud.ply, poses.npy to `tsdf_map/`
- **Load Map** — loads previously saved mesh
- **Reset Map** — clears the TSDF volume
- **Show Mesh** — toggle mesh vs point cloud display
- **Point Size** — adjust point cloud rendering size
- **Update Interval** — seconds between visualization refreshes
