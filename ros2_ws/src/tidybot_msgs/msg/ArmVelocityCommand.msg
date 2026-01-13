# Joint velocity command for a 5-DOF WX200 arm
# Joint order: waist, shoulder, elbow, wrist_angle, wrist_rotate

std_msgs/Header header

# Target velocities in rad/s for each joint
float64[5] joint_velocities

# Duration to apply velocity in seconds (0 = until next command)
float64 duration
