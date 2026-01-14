# =============================================================================
# TidyBot2 Docker Image
# Based on ROS2 Humble with VNC Desktop Access
# =============================================================================
FROM tiryoh/ros2-desktop-vnc:humble

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# =============================================================================
# Install System Dependencies
# =============================================================================
RUN apt-get update && apt-get install -y \
    software-properties-common \
    wget \
    build-essential \
    curl \
    git \
    libgl1-mesa-dev \
    libglfw3-dev \
    libegl1-mesa-dev \
    mesa-utils \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-typesupport-cpp \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Install uv (Python Package Manager) system-wide
# =============================================================================
RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin sh

# Verify uv installation
RUN /usr/local/bin/uv --version

# =============================================================================
# Copy TidyBot2 Repository to shared location
# =============================================================================
# Use /opt for system-wide installation accessible by all users
WORKDIR /opt

# Copy the entire repository
COPY . /opt/tidybot2

# Remove Docker folder and any build artifacts from the copy
# Also fix permissions in one pass
RUN rm -rf /opt/tidybot2/Docker \
    /opt/tidybot2/.venv \
    /opt/tidybot2/ros2_ws/build \
    /opt/tidybot2/ros2_ws/install \
    /opt/tidybot2/ros2_ws/log \
    /opt/tidybot2/.git && \
    chmod -R 755 /opt/tidybot2 && \
    find /opt/tidybot2 -type d -exec chmod 777 {} \;

# =============================================================================
# Setup Python Environment with uv
# =============================================================================
WORKDIR /opt/tidybot2

# Install Python dependencies (with permissive umask so files are created accessible)
RUN umask 000 && /usr/local/bin/uv sync

# =============================================================================
# Build ROS2 Workspace
# =============================================================================
WORKDIR /opt/tidybot2/ros2_ws

# Build with ROS2 environment (with permissive umask)
RUN /bin/bash -c "\
    umask 000 && \
    source /opt/ros/humble/setup.bash && \
    colcon build && \
    echo 'ROS2 workspace built successfully'"

# =============================================================================
# Create launcher scripts in /usr/local/bin
# =============================================================================

# Standalone MuJoCo simulation launcher
RUN echo '#!/bin/bash' > /usr/local/bin/tidybot-sim && \
    echo 'cd /opt/tidybot2/simulation/scripts' >> /usr/local/bin/tidybot-sim && \
    echo 'uv run python test_move.py' >> /usr/local/bin/tidybot-sim && \
    chmod +x /usr/local/bin/tidybot-sim

# ROS2 simulation launcher
RUN echo '#!/bin/bash' > /usr/local/bin/tidybot-ros && \
    echo 'source /opt/ros/humble/setup.bash' >> /usr/local/bin/tidybot-ros && \
    echo 'source /opt/tidybot2/ros2_ws/install/setup.bash' >> /usr/local/bin/tidybot-ros && \
    echo 'export TIDYBOT_REPO_ROOT=/opt/tidybot2' >> /usr/local/bin/tidybot-ros && \
    echo 'export PYTHONPATH="/opt/tidybot2/.venv/lib/python3.10/site-packages:$PYTHONPATH"' >> /usr/local/bin/tidybot-ros && \
    echo 'cd /opt/tidybot2/ros2_ws' >> /usr/local/bin/tidybot-ros && \
    echo 'ros2 launch tidybot_bringup sim.launch.py' >> /usr/local/bin/tidybot-ros && \
    chmod +x /usr/local/bin/tidybot-ros

# ROS2 simulation without RViz
RUN echo '#!/bin/bash' > /usr/local/bin/tidybot-ros-no-rviz && \
    echo 'source /opt/ros/humble/setup.bash' >> /usr/local/bin/tidybot-ros-no-rviz && \
    echo 'source /opt/tidybot2/ros2_ws/install/setup.bash' >> /usr/local/bin/tidybot-ros-no-rviz && \
    echo 'export TIDYBOT_REPO_ROOT=/opt/tidybot2' >> /usr/local/bin/tidybot-ros-no-rviz && \
    echo 'export PYTHONPATH="/opt/tidybot2/.venv/lib/python3.10/site-packages:$PYTHONPATH"' >> /usr/local/bin/tidybot-ros-no-rviz && \
    echo 'cd /opt/tidybot2/ros2_ws' >> /usr/local/bin/tidybot-ros-no-rviz && \
    echo 'ros2 launch tidybot_bringup sim.launch.py use_rviz:=false' >> /usr/local/bin/tidybot-ros-no-rviz && \
    chmod +x /usr/local/bin/tidybot-ros-no-rviz

# Test scripts runner
RUN echo '#!/bin/bash' > /usr/local/bin/tidybot-test-base && \
    echo 'source /opt/ros/humble/setup.bash' >> /usr/local/bin/tidybot-test-base && \
    echo 'source /opt/tidybot2/ros2_ws/install/setup.bash' >> /usr/local/bin/tidybot-test-base && \
    echo 'export PYTHONPATH="/opt/tidybot2/.venv/lib/python3.10/site-packages:$PYTHONPATH"' >> /usr/local/bin/tidybot-test-base && \
    echo 'ros2 run tidybot_bringup test_base.py' >> /usr/local/bin/tidybot-test-base && \
    chmod +x /usr/local/bin/tidybot-test-base

RUN echo '#!/bin/bash' > /usr/local/bin/tidybot-test-arms && \
    echo 'source /opt/ros/humble/setup.bash' >> /usr/local/bin/tidybot-test-arms && \
    echo 'source /opt/tidybot2/ros2_ws/install/setup.bash' >> /usr/local/bin/tidybot-test-arms && \
    echo 'export PYTHONPATH="/opt/tidybot2/.venv/lib/python3.10/site-packages:$PYTHONPATH"' >> /usr/local/bin/tidybot-test-arms && \
    echo 'ros2 run tidybot_bringup test_arms.py' >> /usr/local/bin/tidybot-test-arms && \
    chmod +x /usr/local/bin/tidybot-test-arms

# =============================================================================
# Create desktop shortcuts (for VNC desktop)
# =============================================================================
RUN mkdir -p /usr/share/applications

# MuJoCo Simulation desktop entry
RUN echo '[Desktop Entry]' > /usr/share/applications/tidybot-sim.desktop && \
    echo 'Version=1.0' >> /usr/share/applications/tidybot-sim.desktop && \
    echo 'Type=Application' >> /usr/share/applications/tidybot-sim.desktop && \
    echo 'Name=TidyBot2 MuJoCo Sim' >> /usr/share/applications/tidybot-sim.desktop && \
    echo 'Comment=Run TidyBot2 standalone MuJoCo simulation' >> /usr/share/applications/tidybot-sim.desktop && \
    echo 'Exec=x-terminal-emulator -e tidybot-sim' >> /usr/share/applications/tidybot-sim.desktop && \
    echo 'Icon=utilities-terminal' >> /usr/share/applications/tidybot-sim.desktop && \
    echo 'Terminal=false' >> /usr/share/applications/tidybot-sim.desktop && \
    echo 'Categories=Development;Robotics;' >> /usr/share/applications/tidybot-sim.desktop

# ROS2 Simulation desktop entry
RUN echo '[Desktop Entry]' > /usr/share/applications/tidybot-ros.desktop && \
    echo 'Version=1.0' >> /usr/share/applications/tidybot-ros.desktop && \
    echo 'Type=Application' >> /usr/share/applications/tidybot-ros.desktop && \
    echo 'Name=TidyBot2 ROS2 Sim' >> /usr/share/applications/tidybot-ros.desktop && \
    echo 'Comment=Run TidyBot2 ROS2 simulation with RViz' >> /usr/share/applications/tidybot-ros.desktop && \
    echo 'Exec=x-terminal-emulator -e tidybot-ros' >> /usr/share/applications/tidybot-ros.desktop && \
    echo 'Icon=utilities-terminal' >> /usr/share/applications/tidybot-ros.desktop && \
    echo 'Terminal=false' >> /usr/share/applications/tidybot-ros.desktop && \
    echo 'Categories=Development;Robotics;' >> /usr/share/applications/tidybot-ros.desktop

# =============================================================================
# Setup environment for all users
# =============================================================================
RUN echo "" >> /etc/bash.bashrc && \
    echo "# ==============================================================================" >> /etc/bash.bashrc && \
    echo "# TidyBot2 Environment" >> /etc/bash.bashrc && \
    echo "# ==============================================================================" >> /etc/bash.bashrc && \
    echo "export TIDYBOT_ROOT=/opt/tidybot2" >> /etc/bash.bashrc && \
    echo "export PATH=/usr/local/bin:\$PATH" >> /etc/bash.bashrc && \
    echo "" >> /etc/bash.bashrc && \
    echo "# Source ROS2" >> /etc/bash.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "" >> /etc/bash.bashrc && \
    echo "# Source TidyBot2 workspace" >> /etc/bash.bashrc && \
    echo "source /opt/tidybot2/ros2_ws/install/setup.bash 2>/dev/null || true" >> /etc/bash.bashrc && \
    echo "" >> /etc/bash.bashrc && \
    echo 'echo ""' >> /etc/bash.bashrc && \
    echo 'echo "=========================================="' >> /etc/bash.bashrc && \
    echo 'echo "  TidyBot2 Ready!"' >> /etc/bash.bashrc && \
    echo 'echo "=========================================="' >> /etc/bash.bashrc && \
    echo 'echo "Commands:"' >> /etc/bash.bashrc && \
    echo 'echo "  tidybot-sim          - MuJoCo standalone"' >> /etc/bash.bashrc && \
    echo 'echo "  tidybot-ros          - ROS2 + RViz"' >> /etc/bash.bashrc && \
    echo 'echo "  tidybot-ros-no-rviz  - ROS2 only"' >> /etc/bash.bashrc && \
    echo 'echo ""' >> /etc/bash.bashrc && \
    echo 'echo "Test scripts (run in 2nd terminal):"' >> /etc/bash.bashrc && \
    echo 'echo "  tidybot-test-base    - Test base movement"' >> /etc/bash.bashrc && \
    echo 'echo "  tidybot-test-arms    - Test arm control"' >> /etc/bash.bashrc && \
    echo 'echo "=========================================="' >> /etc/bash.bashrc && \
    echo 'echo ""' >> /etc/bash.bashrc

# =============================================================================
# Final Setup
# =============================================================================
ENV PATH="/usr/local/bin:${PATH}"
ENV TIDYBOT_ROOT="/opt/tidybot2"

# Expose VNC port
EXPOSE 80

# Working directory
WORKDIR /opt/tidybot2

# Inherit entrypoint from base image (starts VNC server)
