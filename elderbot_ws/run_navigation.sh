#!/bin/bash
# ElderBot - Run Navigation (with pre-built map)
# Usage: bash run_navigation.sh [map_path] [initial_x] [initial_y] [initial_yaw] [extra ros2 launch args...]

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2
source /opt/ros/humble/setup.bash

# Source ElderBot workspace
source "$SCRIPT_DIR/install/setup.bash"

MAP_PATH="${1:-$HOME/maps/my_map1.yaml}"
INIT_X="0.0"
INIT_Y="0.0"
INIT_YAW="0.0"
EXTRA_LAUNCH_ARGS=()
PUBLISH_INITIAL_POSE="false"

if [ "$#" -ge 4 ] && [[ "${2}" != *:=* ]] && [[ "${3}" != *:=* ]] && [[ "${4}" != *:=* ]]; then
    INIT_X="${2}"
    INIT_Y="${3}"
    INIT_YAW="${4}"
    EXTRA_LAUNCH_ARGS=("${@:5}")
    PUBLISH_INITIAL_POSE="true"
else
    EXTRA_LAUNCH_ARGS=("${@:2}")
fi

echo "=========================================="
echo "  ElderBot Navigation"
echo "  Map: $MAP_PATH"
echo "  Initial Pose: x=$INIT_X, y=$INIT_Y, yaw=$INIT_YAW"
echo "=========================================="

ros2 launch elderbot_bringup bringup.launch.py \
    map:="$MAP_PATH" \
    initial_pose_x:="$INIT_X" \
    initial_pose_y:="$INIT_Y" \
    initial_pose_yaw:="$INIT_YAW" \
    publish_initial_pose:="$PUBLISH_INITIAL_POSE" \
    "${EXTRA_LAUNCH_ARGS[@]}"
