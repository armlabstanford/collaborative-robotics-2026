# End-effector Cartesian velocity command
# Controls the velocity of the arm's end-effector in task space

std_msgs/Header header

# Linear velocity (m/s) and angular velocity (rad/s)
geometry_msgs/Twist twist

# Reference frame for the velocity command
# Options: "base_link" (world-aligned) or "end_effector" (tool-aligned)
string frame_id

# Duration to apply velocity in seconds (0 = until next command)
float64 duration
