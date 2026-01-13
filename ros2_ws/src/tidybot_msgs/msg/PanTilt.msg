# Pan-tilt camera gimbal command
# Controls the RealSense D435 camera orientation

std_msgs/Header header

# Pan angle in radians
# Range: [-1.57, 1.57] (approximately +/- 90 degrees)
float64 pan

# Tilt angle in radians
# Range: [-1.57, 1.31] (approximately -90 to +75 degrees)
float64 tilt
