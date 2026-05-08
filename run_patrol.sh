#!/bin/bash
# ElderBot - 巡逻导航 (Patrol Navigation Action Client)
# 使用方法:
#   bash run_patrol.sh                          # 使用默认巡逻点配置
#   bash run_patrol.sh /path/to/waypoints.yaml  # 使用自定义巡逻点文件
#
# 注意: 运行前需要先启动导航栈:
#   bash run_navigation.sh ~/maps/my_map2.yaml
# 然后在另一个终端运行此脚本

set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Source ElderBot workspace
source ~/elderbot_ws/install/setup.bash

# 巡逻点配置文件 (默认直接读取 src 目录下的文件，修改后无需编译即可生效)
WAYPOINTS_FILE="${1:-$HOME/elderbot_ws/src/elderbot_navigation/config/patrol_waypoints.yaml}"

echo "=========================================="
echo "  ElderBot 巡逻导航"
echo "  巡逻点配置: $WAYPOINTS_FILE"
echo "=========================================="
echo ""
echo "提示: 请确保导航栈已启动 (bash run_navigation.sh)"
echo "按 Ctrl+C 可停止巡逻并关闭 action server"
echo ""

cleanup() {
    if [[ -n "${SERVER_PID:-}" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
        kill "$SERVER_PID" 2>/dev/null || true
        wait "$SERVER_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

ros2 run elderbot_navigation patrol_node.py \
    --ros-args --params-file "$WAYPOINTS_FILE" &
SERVER_PID=$!

for _ in $(seq 1 20); do
    if ros2 action list | grep -qx "/patrol"; then
        break
    fi
    sleep 0.5
done

if ! ros2 action list | grep -qx "/patrol"; then
    echo "错误: /patrol action server 未成功启动"
    exit 1
fi

ros2 action send_goal --feedback /patrol elderbot_navigation/action/Patrol \
    "{loop_forever: true}"
