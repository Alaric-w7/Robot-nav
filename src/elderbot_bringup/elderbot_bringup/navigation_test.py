import rclpy
import sys
import argparse
import math
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.time import Time

"""
使用说明:
1. 确保已启动 Nav2 导航栈 (通常是 navigation_launch.py 或类似于 bringup 的 launch 文件)
2. 可以通过命令行参数指定起点和终点:
   ros2 run elderbot_bringup navigation_test
   或者:
   ros2 run elderbot_bringup navigation_test -- --sx -0.2 --sy 1.1 --syaw 0.0 --gx 2.8 --gy 3.4 --gyaw 0.0
"""

def euler_to_quaternion(yaw):
    """
    将欧拉角 yaw (弧度) 转换为四元数
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def main():
    parser = argparse.ArgumentParser(description='Nav2 导航测试脚本')
    # 默认值更新为用户提供的坐标
    parser.add_argument('--sx', type=float, default=-0.224, help='起始点 X 坐标')
    parser.add_argument('--sy', type=float, default=1.11, help='起始点 Y 坐标')
    parser.add_argument('--syaw', type=float, default=0.0181, help='起始点 Yaw 角度 (弧度)')

    parser.add_argument('--gx', type=float, default=2.89, help='目标点 X 坐标')
    parser.add_argument('--gy', type=float, default=3.46, help='目标点 Y 坐标')
    parser.add_argument('--gyaw', type=float, default=0.00296, help='目标点 Yaw 角度 (弧度)')

    # 简单过滤 ROS 参数
    argv = [arg for arg in sys.argv[1:] if not arg.startswith('--ros-args')]
    args, _ = parser.parse_known_args(argv)

    rclpy.init()

    # 实例化 BasicNavigator
    navigator = BasicNavigator()

    # ==============================
    # 1. 设定初始位姿 (Start Point)
    # ==============================
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()

    initial_pose.pose.position.x = args.sx
    initial_pose.pose.position.y = args.sy
    initial_pose.pose.orientation = euler_to_quaternion(args.syaw)

    print(f"设置初始位姿: x={args.sx}, y={args.sy}, yaw={args.syaw}")
    navigator.setInitialPose(initial_pose)

    # 等待 Nav2 启动完成
    print("等待 Navigation2 激活...")
    navigator.waitUntilNav2Active()

    # ==============================
    # 2. 设定目标点 (Target Point)
    # ==============================
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    goal_pose.pose.position.x = args.gx
    goal_pose.pose.position.y = args.gy
    goal_pose.pose.orientation = euler_to_quaternion(args.gyaw)

    print(f"正在导航到目标: x={args.gx}, y={args.gy}, yaw={args.gyaw}")

    # 发送导航目标
    navigator.goToPose(goal_pose)

    # ==============================
    # 3. 监控过程并打印日志
    # ==============================
    i = 0
    last_distance = float('inf')
    stuck_counter = 0

    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'[INFO] 距离目标还有: {feedback.distance_remaining:.2f} 米, '
                  f'导航已用时间: {Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9:.2f} 秒')

            # 简单的卡住检测逻辑
            if abs(feedback.distance_remaining - last_distance) < 0.05:
                stuck_counter += 1
            else:
                stuck_counter = 0

            last_distance = feedback.distance_remaining

            if stuck_counter > 20:
                 print("[WARN] 机器人似乎卡住了或移动非常缓慢...")

    # ==============================
    # 4. 分析结果
    # ==============================
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('[SUCCESS] 导航成功到达目的地！')
    elif result == TaskResult.CANCELED:
        print('[CANCELED] 导航任务被取消！')
    elif result == TaskResult.FAILED:
        print('[FAILED] 导航失败！')
        print('--------------------------------------------------')
        print('可能原因分析:')
        print(f'1. 目标点 ({args.gx}, {args.gy}) 可能在地图障碍物内或在膨胀层(Inflation Layer)中。')
        print(f'2. 初始位姿 ({args.sx}, {args.sy}) 设置后，机器人可能发现自己处于致命成本(Lethal Cost)区域。')
        print(f'3. 全局路径规划器无法规划出路径。')
        print('建议操作:')
        print('   - 在 Rviz 中勾选 "Global Costmap" -> "Costmap" -> "Show Costmap"，查看目标点是否被深色/彩色障碍物覆盖。')
        print('   - 尝试稍微移动目标点坐标。')
        print('--------------------------------------------------')
    else:
        print('[UNKNOWN] 导航结果未知')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
