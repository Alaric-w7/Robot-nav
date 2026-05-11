#!/bin/bash
# ElderBot - Run RTAB-Map RGB-D + LiDAR mapping/localization
# Usage:
#   Mapping:      bash run_rtabmap.sh [database_path] [rviz] [rtabmap_viz]
#   Localization: bash run_rtabmap.sh [database_path] [rviz] [rtabmap_viz] localization ["x y z roll pitch yaw"]

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source Orbbec camera workspace if available
if [ -f ~/orbbec_ws/install/setup.bash ]; then
    source ~/orbbec_ws/install/setup.bash
fi

# Prefer the current workspace, then fall back to the legacy elderbot workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
elif [ -f ~/elderbot_ws/install/setup.bash ]; then
    source ~/elderbot_ws/install/setup.bash
else
    echo "No workspace setup.bash found. Please build the workspace first."
    exit 1
fi

DB_PATH="${1:-$HOME/maps/elderbot_rtabmap.db}"
RVIZ="${2:-false}"
RTABMAP_VIZ="${3:-false}"
MODE="${4:-mapping}"
INITIAL_POSE="${5:-}"

case "$MODE" in
    mapping|map|slam)
        LOCALIZATION="false"
        FRESH_DB="true"
        ;;
    localization|localize|relocalization|relocalize)
        LOCALIZATION="true"
        FRESH_DB="false"
        ;;
    *)
        echo "Unknown mode: $MODE"
        echo "Use 'mapping' or 'localization'."
        exit 1
        ;;
esac

mkdir -p "$(dirname "$DB_PATH")"

echo "=========================================="
echo "  ElderBot RTAB-Map"
echo "  Mode: $MODE"
echo "  Database: $DB_PATH"
echo "  RViz: $RVIZ"
echo "  RTAB-Map GUI: $RTABMAP_VIZ"
echo "  Fresh DB: $FRESH_DB"
if [ -n "$INITIAL_POSE" ]; then
    echo "  Initial pose: $INITIAL_POSE"
fi
if [ "$LOCALIZATION" = "false" ]; then
    echo "  Save 2D map: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map_rtabmap"
fi
echo "=========================================="

LAUNCH_ARGS=(
    "database_path:=$DB_PATH"
    "rviz:=$RVIZ"
    "rtabmap_viz:=$RTABMAP_VIZ"
    "fresh_db:=$FRESH_DB"
    "localization:=$LOCALIZATION"
)

if [ -n "$INITIAL_POSE" ]; then
    LAUNCH_ARGS+=("initial_pose:=$INITIAL_POSE")
fi

ros2 launch elderbot_bringup rtabmap.launch.py "${LAUNCH_ARGS[@]}"
