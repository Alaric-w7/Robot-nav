#!/bin/bash
# ElderBot - Run Navigation (with pre-built map)
# Usage: bash run_navigation.sh [map_path] [initial_x] [initial_y] [initial_yaw]

set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Source Orbbec camera workspace if available
if [ -f ~/orbbec_ws/install/setup.bash ]; then
    source ~/orbbec_ws/install/setup.bash
fi

# Source ElderBot workspace
source ~/elderbot_ws/install/setup.bash

MAP_PATH="${1:-$HOME/maps/my_map1.yaml}"
INIT_X="${2:-0.0}"
INIT_Y="${3:-0.0}"
INIT_YAW="${4:-0.0}"

echo "=========================================="
echo "  ElderBot Navigation"
echo "  Map: $MAP_PATH"
echo "  Initial Pose: x=$INIT_X, y=$INIT_Y, yaw=$INIT_YAW"
echo "=========================================="

ros2 launch elderbot_bringup bringup.launch.py \
    map:="$MAP_PATH" \
    initial_pose_x:="$INIT_X" \
    initial_pose_y:="$INIT_Y" \
    initial_pose_yaw:="$INIT_YAW"
