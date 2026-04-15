#!/bin/bash
# ElderBot - Run SLAM Mapping
# Usage: bash run_slam.sh
# After mapping, save map with: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Source Orbbec camera workspace if available
if [ -f ~/orbbec_ws/install/setup.bash ]; then
    source ~/orbbec_ws/install/setup.bash
fi

# Source ElderBot workspace
source ~/elderbot_ws/install/setup.bash

echo "=========================================="
echo "  ElderBot SLAM Mapping"
echo "  Use teleop to drive the robot around"
echo "  Save map: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map"
echo "=========================================="

ros2 launch elderbot_bringup slam.launch.py
