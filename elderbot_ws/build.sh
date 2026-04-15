#!/bin/bash
# ElderBot Workspace Build Script
# Usage: bash build.sh [package_name]

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

# Source ROS2
source /opt/ros/humble/setup.bash

# Source Orbbec camera workspace if available
if [ -f ~/orbbec_ws/install/setup.bash ]; then
    source ~/orbbec_ws/install/setup.bash
fi

if [ -n "$1" ]; then
    echo "Building package: $1"
    colcon build --packages-select "$1" --symlink-install
else
    echo "Building all packages..."
    colcon build --symlink-install
fi

echo ""
echo "Build complete!"
echo "Run: source ~/elderbot_ws/install/setup.bash"
