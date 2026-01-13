# Gripper command for Robotiq 2F-85 gripper
# Normalized position control

std_msgs/Header header

# Gripper position: 0.0 = fully open, 1.0 = fully closed
float64 position

# Optional effort limit: 0.0 = minimum force, 1.0 = maximum force
# Default to 0.5 if not specified
float64 effort
