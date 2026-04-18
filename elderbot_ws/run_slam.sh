#!/bin/bash
# ElderBot - Run SLAM Mapping
# Usage: bash run_slam.sh [extra ros2 launch args...]
# After mapping, save map with: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2
source /opt/ros/humble/setup.bash

# Source ElderBot workspace
source "$SCRIPT_DIR/install/setup.bash"

EXTRA_LAUNCH_ARGS=("$@")

echo "=========================================="
echo "  ElderBot SLAM Mapping"
echo "  Use teleop to drive the robot around"
echo "  Save map: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map"
echo "=========================================="

ros2 launch elderbot_bringup slam.launch.py "${EXTRA_LAUNCH_ARGS[@]}"
