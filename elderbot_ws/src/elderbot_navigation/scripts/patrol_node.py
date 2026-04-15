#!/usr/bin/env python3
"""
巡逻导航节点 (Patrol Navigation Node)

功能：
  - 循环导航至配置文件中预设的巡逻点
  - 在每个点停留 wait_duration 设定的时间
  - 若导航长时间未到达或失败，超时跳过
"""

import math
import time
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import BatteryState

def euler_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.declare_parameter('wait_duration', 3.0)
        self.declare_parameter('navigation_timeout', 120.0)
        self.declare_parameter('waypoint_names', ['point_a'])
        self.wait_duration = self.get_parameter('wait_duration').value
        self.nav_timeout = self.get_parameter('navigation_timeout').value
        waypoint_names = self.get_parameter('waypoint_names').value

        self.waypoints = []
        for name in waypoint_names:
            self.declare_parameter(f'{name}.x', 0.0)
            self.declare_parameter(f'{name}.y', 0.0)
            self.declare_parameter(f'{name}.yaw', 0.0)
            x = self.get_parameter(f'{name}.x').value
            y = self.get_parameter(f'{name}.y').value
            yaw = self.get_parameter(f'{name}.yaw').value
            self.waypoints.append({'name': name, 'x': x, 'y': y, 'yaw': yaw})

        self.declare_parameter('battery_threshold', 0.20)
        self.declare_parameter('battery_resume', 0.90)
        self.battery_threshold = self.get_parameter('battery_threshold').value
        self.battery_resume = self.get_parameter('battery_resume').value
        self.current_battery = 1.0
        self.is_paused_for_charge = False

        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)

        if not self.waypoints:
            self.get_logger().error('没有配置巡逻点！')
            sys.exit(1)

        self.get_logger().info('==================================================')
        self.get_logger().info('  巡逻导航节点启动')
        self.get_logger().info(f'  巡逻点数量: {len(self.waypoints)}')
        self.get_logger().info(f'  每点停留时间: {self.wait_duration}s')
        self.get_logger().info(f'  导航超时时间: {self.nav_timeout}s')
        self.get_logger().info('  巡逻点列表:')
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'    [{i+1}] {wp["name"]}: x={wp["x"]:.2f}, y={wp["y"]:.2f}, yaw={wp["yaw"]:.2f}')
        self.get_logger().info('==================================================')

    def create_goal_pose(self, waypoint: dict) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(waypoint['x'])
        goal.pose.position.y = float(waypoint['y'])
        goal.pose.position.z = 0.0
        goal.pose.orientation = euler_to_quaternion(float(waypoint['yaw']))
        return goal

    def battery_callback(self, msg: BatteryState):
        val = msg.percentage
        if val > 1.0:
            val = val / 100.0
        self.current_battery = val

        if not self.is_paused_for_charge and self.current_battery < self.battery_threshold:
            self.is_paused_for_charge = True
            self.get_logger().warn(f'>>> 注意：电量({self.current_battery*100:.1f}%)过低！暂停巡逻等待充电 <<<')
        elif self.is_paused_for_charge and self.current_battery >= self.battery_resume:
            self.is_paused_for_charge = False
            self.get_logger().info(f'>>> 电量({self.current_battery*100:.1f}%)已恢复！继续巡逻 <<<')

    def run_patrol(self):
        navigator = BasicNavigator()
        self.get_logger().info('等待 Navigation2 激活...')
        navigator.waitUntilNav2Active()

        round_count = 0
        try:
            while rclpy.ok():
                round_count += 1
                self.get_logger().info(f'\n>>> 第 {round_count} 轮巡逻开始 <<<')

                for i, waypoint in enumerate(self.waypoints):
                    if not rclpy.ok(): break
                    
                    while self.is_paused_for_charge and rclpy.ok():
                        self.get_logger().info(f'巡逻已挂起，当前电量: {self.current_battery*100:.1f}%, 充至 {self.battery_resume*100:.1f}% 后恢复', throttle_duration_sec=10.0)
                        time.sleep(2.0)

                    if not rclpy.ok(): break

                    self.get_logger().info(f'[{i+1}/{len(self.waypoints)}] 正在导航到 {waypoint["name"]} (x={waypoint["x"]:.2f}, y={waypoint["y"]:.2f})')
                    goal_pose = self.create_goal_pose(waypoint)
                    navigator.goToPose(goal_pose)
                    start_time = time.time()
                    
                    while not navigator.isTaskComplete():
                        if not rclpy.ok():
                            navigator.cancelTask()
                            return
                        elapsed = time.time() - start_time
                        if elapsed > self.nav_timeout:
                            self.get_logger().warn(f'  ✗ 导航超时！跳过')
                            navigator.cancelTask()
                            while not navigator.isTaskComplete(): pass
                            break

                    result = navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info(f'  ✓ 成功到达 {waypoint["name"]}！')
                        if self.wait_duration > 0:
                            self.get_logger().info(f'  停留 {self.wait_duration}s...')
                            time.sleep(self.wait_duration)
                    elif result == TaskResult.FAILED:
                        self.get_logger().warn(f'  ✗ 导航到 {waypoint["name"]} 失败，跳过继续下一个点')

                self.get_logger().info(f'>>> 第 {round_count} 轮巡逻完成，开始下一轮... <<<\n')
                
        except KeyboardInterrupt:
            self.get_logger().info('收到 Ctrl+C，停止巡逻...')
            navigator.cancelTask()

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_patrol()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
