#!/bin/bash
# run_autodock.sh - 启动独立的自动回充监控程序

# 定位到工作空间目录
cd ~/elderbot_ws

# Source 环境
source /opt/ros/humble/setup.bash
source ~/elderbot_ws/install/setup.bash

echo "=========================================="
echo "  启动独立自动回充监控服务 (Auto Docking Monitor)"
echo "  说明: 该服务会在后台静默运行，"
echo "  一旦检测到低电量，会切断所有导航命令并寻找对接点。"
echo "  同时会提供 /auto_dock action 供外部强制触发回充。"
echo "=========================================="

# 直接运行 Python 脚本，并透传可能附加的参数 (如 --ros-args)
python3 src/elderbot_navigation/scripts/auto_dock_node.py "$@"
