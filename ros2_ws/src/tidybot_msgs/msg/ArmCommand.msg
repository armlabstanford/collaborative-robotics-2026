# Target joint positions for a 5-DOF WX200 arm
# Joint order: waist, shoulder, elbow, wrist_angle, wrist_rotate

std_msgs/Header header

# Target positions in radians for each joint
float64[5] joint_positions

# Time to reach target in seconds (0 = immediate/as fast as possible)
float64 duration
