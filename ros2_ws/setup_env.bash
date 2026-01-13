#!/bin/bash
# Setup script for TidyBot2 ROS2 workspace
# This combines the ROS2 environment with the uv-managed packages
#
# Usage: source setup_env.bash
#
# NOTE: We DON'T activate the venv (which would change the python binary).
# Instead, we add the venv's site-packages to PYTHONPATH so ROS2's Python
# can import packages like mujoco, mink, numpy, etc.

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Setting up TidyBot2 ROS2 environment..."
echo ""

# 0. Deactivate any active venv and remove .venv from PATH
#    This ensures colcon uses system Python, not venv Python
if [ -n "$VIRTUAL_ENV" ]; then
    deactivate 2>/dev/null
    echo "✓ Deactivated existing virtual environment"
fi
# Remove any .venv/bin from PATH (in case it was added manually)
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "\.venv/bin" | tr '\n' ':' | sed 's/:$//')

# 1. Source ROS2 Humble (uses system Python with ROS2 packages)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ Sourced ROS2 Humble"
else
    echo "✗ ROS2 Humble not found at /opt/ros/humble"
    echo "  Install with: sudo apt install ros-humble-desktop"
    return 1
fi

# 2. Add uv venv site-packages to PYTHONPATH (for mujoco, mink, numpy, etc.)
#    This lets ROS2's Python import your uv-managed packages WITHOUT
#    changing which python binary is used (which would break colcon build)
UV_SITE_PACKAGES="$PROJECT_ROOT/.venv/lib/python3.10/site-packages"
if [ -d "$UV_SITE_PACKAGES" ]; then
    export PYTHONPATH="$UV_SITE_PACKAGES:$PYTHONPATH"
    echo "✓ Added uv packages to PYTHONPATH"
else
    echo "⚠ uv environment not found at $UV_SITE_PACKAGES"
    echo "  Run 'cd $PROJECT_ROOT && uv sync' to create it"
fi

# 3. Set repo root for finding simulation assets
export TIDYBOT_REPO_ROOT="$PROJECT_ROOT"
echo "✓ Set TIDYBOT_REPO_ROOT=$PROJECT_ROOT"

# 4. Source the ROS2 workspace if built
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
    echo "✓ Sourced ROS2 workspace (ros2_ws)"
else
    echo "⚠ Workspace not built yet. Run:"
    echo "  cd $SCRIPT_DIR && colcon build"
fi

# 5. Source install/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
echo "✓ Sourced install/setup.bash"

echo ""
echo "Environment ready!"
echo "  Python: $(which python3) (ROS2 system Python)"
echo "  uv packages available via PYTHONPATH"
echo ""
