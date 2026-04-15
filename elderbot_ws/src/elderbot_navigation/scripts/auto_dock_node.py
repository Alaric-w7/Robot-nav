#!/usr/bin/env python3
"""
自动回充监控节点 (Auto Dock Node) - V3 自适应版

核心逻辑（两个独立职责）：
  职责 A：检测到开始充电 → TF2 记录充电桩位置 → 计算回充准备点 → 保存
  职责 B：电量 < 20% → 导航到回充准备点 → 倒车检测充电
"""

import math
import time
import threading
import subprocess
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import BatteryState

import tf2_ros
from tf2_ros import TransformException


def euler_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def lifecycle_set_state(node_name, transition):
    cmd = (f"bash -c 'source /opt/ros/humble/setup.bash && "
           f"ros2 lifecycle set /{node_name} {transition}'")
    try:
        subprocess.run(cmd, shell=True, capture_output=True,
                       text=True, timeout=8)
    except Exception:
        pass


class AutoDockNode(Node):
    def __init__(self):
        super().__init__('auto_dock_node')

        # ==================== 参数 ====================
        self.declare_parameter('battery_threshold', 0.20)
        self.declare_parameter('backup_speed', 0.3)
        self.declare_parameter('backup_max_dist', 0.35)
        self.declare_parameter('prep_distance', 0.30)
        self.declare_parameter('timeout', 30.0)
        self.declare_parameter('default_charger_x', -0.77)
        self.declare_parameter('default_charger_y', 1.17)
        self.declare_parameter('default_charger_yaw', 2.36)

        self.battery_threshold = self.get_parameter('battery_threshold').value
        self.backup_speed = self.get_parameter('backup_speed').value
        self.backup_max_dist = self.get_parameter('backup_max_dist').value
        self.prep_distance = self.get_parameter('prep_distance').value
        self.timeout = self.get_parameter('timeout').value
        self.default_charger_x = self.get_parameter('default_charger_x').value
        self.default_charger_y = self.get_parameter('default_charger_y').value
        self.default_charger_yaw = self.get_parameter(
            'default_charger_yaw').value

        # dock_pose.yaml 路径
        self.dock_pose_file = os.path.expanduser(
            '~/elderbot_ws/src/elderbot_navigation/config/dock_pose.yaml')

        # ==================== 状态 ====================
        self.current_battery = 1.0
        self.current_current = 0.0
        self.is_charging = False
        self.was_charging = False
        self.is_docking = False
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # ==================== TF2 ====================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ==================== 话题 ====================
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 确保 dock_pose.yaml 存在
        if not os.path.exists(self.dock_pose_file):
            self._create_default_dock_pose()

        dock = self.load_dock_pose()
        self.get_logger().info('=' * 50)
        self.get_logger().info('  自动回充 (V3 自适应版)')
        self.get_logger().info(
            f'  低电量阈值: <{self.battery_threshold*100:.0f}%')
        if dock:
            self.get_logger().info(
                f'  充电桩位置: ({dock["charger_x"]}, '
                f'{dock["charger_y"]}) yaw={dock["charger_yaw"]}')
            self.get_logger().info(
                f'  回充准备点: ({dock["prep_x"]}, '
                f'{dock["prep_y"]}) yaw={dock["prep_yaw"]}')
        self.get_logger().info(
            f'  倒车: 最大{self.backup_max_dist*100:.0f}cm '
            f'@ {self.backup_speed}m/s')
        self.get_logger().info(f'  配置文件: {self.dock_pose_file}')
        self.get_logger().info('=' * 50)

    # ==================== 回调 ====================

    def battery_callback(self, msg: BatteryState):
        val = msg.percentage
        if val > 1.0:
            val = val / 100.0
        self.current_battery = val
        self.current_current = msg.current
        self.is_charging = msg.current > 0.1

    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    # ==================== 工具方法 ====================

    def get_map_pose(self):
        """通过 TF2 获取机器人在 map 坐标系下的位姿"""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = yaw_from_quaternion(t.transform.rotation)
            return x, y, yaw
        except TransformException as e:
            self.get_logger().warn(f'TF 查询失败: {e}')
            return None

    def _create_default_dock_pose(self):
        """用默认参数创建初始 dock_pose.yaml"""
        cx = self.default_charger_x
        cy = self.default_charger_y
        cyaw = self.default_charger_yaw
        px = cx + self.prep_distance * math.cos(cyaw)
        py = cy + self.prep_distance * math.sin(cyaw)
        self.save_dock_pose(cx, cy, cyaw, px, py, cyaw)
        self.get_logger().info(
            f'首次启动，已创建默认 dock_pose: '
            f'charger=({cx}, {cy}), prep=({px:.4f}, {py:.4f})')

    def save_dock_pose(self, cx, cy, cyaw, px, py, pyaw):
        """保存充电桩位置 + 回充准备点到 yaml"""
        data = {
            'charger_x': round(float(cx), 4),
            'charger_y': round(float(cy), 4),
            'charger_yaw': round(float(cyaw), 4),
            'prep_x': round(float(px), 4),
            'prep_y': round(float(py), 4),
            'prep_yaw': round(float(pyaw), 4),
        }
        os.makedirs(os.path.dirname(self.dock_pose_file), exist_ok=True)
        with open(self.dock_pose_file, 'w') as f:
            f.write('# 自动回充配置 - 充电时自动记录\n')
            f.write(f'# 最后更新: {time.strftime("%Y-%m-%d %H:%M:%S")}\n\n')
            f.write('# 充电桩位置（机器人在充电桩上的坐标）\n')
            f.write(f'charger_x: {data["charger_x"]}\n')
            f.write(f'charger_y: {data["charger_y"]}\n')
            f.write(f'charger_yaw: {data["charger_yaw"]}\n\n')
            f.write('# 回充准备点（充电桩前方30cm，导航目标）\n')
            f.write(f'prep_x: {data["prep_x"]}\n')
            f.write(f'prep_y: {data["prep_y"]}\n')
            f.write(f'prep_yaw: {data["prep_yaw"]}\n')
        self.get_logger().info(
            f'   已保存: 充电桩({cx:.4f}, {cy:.4f}, yaw={cyaw:.4f}) '
            f'准备点({px:.4f}, {py:.4f}, yaw={pyaw:.4f})')

    def load_dock_pose(self):
        """读取 dock_pose.yaml"""
        try:
            with open(self.dock_pose_file, 'r') as f:
                data = yaml.safe_load(f)
            return data
        except Exception as e:
            self.get_logger().warn(f'读取 dock_pose 失败: {e}')
            return None

    def create_goal_pose(self, x, y, yaw) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        goal.pose.orientation = euler_to_quaternion(float(yaw))
        return goal

    def full_stop(self):
        twist = Twist()
        for _ in range(30):
            self.cmd_pub.publish(twist)
            time.sleep(0.02)

    def deactivate_nav2(self):
        self.get_logger().info('   [Nav2] 去激活 controller + smoother...')
        lifecycle_set_state('controller_server', 'deactivate')
        lifecycle_set_state('velocity_smoother', 'deactivate')
        time.sleep(0.5)
        self.get_logger().info('   [Nav2] 已去激活 ✓')

    def activate_nav2(self):
        self.get_logger().info('   [Nav2] 恢复 controller + smoother...')
        lifecycle_set_state('velocity_smoother', 'activate')
        lifecycle_set_state('controller_server', 'activate')
        time.sleep(0.5)
        self.get_logger().info('   [Nav2] 已恢复 ✓')

    # ==================== 运动控制 ====================

    def backup_until_charged(self, max_dist, speed):
        """里程计闭环倒车，检测到充电立刻停"""
        start_x = self.odom_x
        start_y = self.odom_y
        t0 = time.time()
        charged = False

        twist = Twist()
        twist.linear.x = -speed

        while rclpy.ok():
            dist = math.sqrt(
                (self.odom_x - start_x) ** 2 +
                (self.odom_y - start_y) ** 2)

            if self.current_current > 0.1:
                charged = True
                self.get_logger().info(
                    f'   ⚡ 检测到充电! (电流: {self.current_current:.2f}A) '
                    f'立刻停车')
                break

            if dist >= max_dist:
                self.get_logger().warn(
                    f'   已倒车 {dist*100:.1f}cm (最大 '
                    f'{max_dist*100:.0f}cm)，未检测到充电')
                break

            if time.time() - t0 > self.timeout:
                self.get_logger().warn(f'   倒车超时 ({self.timeout}s)')
                break

            self.cmd_pub.publish(twist)
            time.sleep(0.02)

        self.full_stop()
        final_dist = math.sqrt(
            (self.odom_x - start_x) ** 2 +
            (self.odom_y - start_y) ** 2)
        elapsed = time.time() - t0
        status = '✓ 已充电' if charged else '✗ 未充电'
        self.get_logger().info(
            f'   倒车完成 {status} 实际: {final_dist*100:.1f}cm '
            f'用时: {elapsed:.1f}s')
        return charged

    # ==================== 职责 A：检测充电 → 记录位置 ====================

    def handle_start_charging(self):
        """检测到开始充电 → 记录充电桩位置 + 计算回充准备点"""
        self.get_logger().info('=' * 40)
        self.get_logger().info('>>> 检测到充电，记录充电桩位置 <<<')

        # 等待 3 秒让 AMCL 定位稳定
        self.get_logger().info('-> 等待 3 秒让定位稳定...')
        time.sleep(3.0)

        # 再次确认仍在充电
        if not self.is_charging:
            self.get_logger().info('充电已断开，取消记录（可能是瞬间接触）')
            return

        # 查询 TF 获取 map 坐标
        pose = self.get_map_pose()
        if pose is None:
            self.get_logger().error('   TF 查询失败，无法记录位置！')
            return

        charger_x, charger_y, charger_yaw = pose
        self.get_logger().info(
            f'-> 充电桩位置: x={charger_x:.4f}, y={charger_y:.4f}, '
            f'yaw={charger_yaw:.4f}')

        # 计算前方 30cm 处的回充准备点
        prep_x = charger_x + self.prep_distance * math.cos(charger_yaw)
        prep_y = charger_y + self.prep_distance * math.sin(charger_yaw)
        prep_yaw = charger_yaw  # 方向不变，车头背对充电桩

        self.get_logger().info(
            f'-> 回充准备点: x={prep_x:.4f}, y={prep_y:.4f}, '
            f'yaw={prep_yaw:.4f}')

        # 保存
        self.save_dock_pose(
            charger_x, charger_y, charger_yaw,
            prep_x, prep_y, prep_yaw)

        self.get_logger().info('>>> 位置记录完成（零打扰，继续充电）<<<')
        self.get_logger().info('=' * 40)

    # ==================== 职责 B：低电量 → 回充 ====================

    def handle_low_battery(self, navigator):
        """低电量 → 导航到回充准备点 → 倒车对接充电桩"""
        self.is_docking = True
        self.get_logger().warn(
            f'!!! 低电量 ({self.current_battery*100:.1f}%) → 启动回充 !!!')

        # 读取 dock_pose
        dock = self.load_dock_pose()
        if dock is None:
            self.get_logger().error(
                'dock_pose.yaml 不存在或无法读取！回充取消。')
            self.is_docking = False
            return

        prep_x = dock['prep_x']
        prep_y = dock['prep_y']
        prep_yaw = dock['prep_yaw']

        # 1. 取消当前导航任务
        navigator.cancelTask()
        while not navigator.isTaskComplete() and rclpy.ok():
            time.sleep(0.1)

        # 2. 导航到回充准备点
        self.get_logger().info(
            f'-> 1. 导航到回充准备点 '
            f'({prep_x}, {prep_y}, yaw={prep_yaw})...')
        pre_pose = self.create_goal_pose(prep_x, prep_y, prep_yaw)
        navigator.goToPose(pre_pose)
        while rclpy.ok() and not navigator.isTaskComplete():
            time.sleep(1.0)
        if navigator.getResult() != TaskResult.SUCCEEDED:
            self.get_logger().error('-> 准备点不可达！回充取消。')
            self.is_docking = False
            return

        self.get_logger().info('-> 到达准备点 ✓，等 2 秒刹车...')
        time.sleep(2.0)

        # 3. 去激活 Nav2
        self.deactivate_nav2()

        # 4. 倒车检测充电
        self.get_logger().info(
            f'-> 2. 倒车最多 {self.backup_max_dist*100:.0f}cm '
            f'(检测充电电流)...')
        charged = self.backup_until_charged(
            self.backup_max_dist, self.backup_speed)

        if charged:
            self.get_logger().info('-> ⚡ 对接成功！已接通充电 ⚡')
        else:
            self.get_logger().warn('-> 对接未成功，未检测到充电电流')

        # 5. 恢复 Nav2
        self.activate_nav2()

        self.get_logger().info('-> 回充流程结束')
        self.is_docking = False

    # ==================== 主循环 ====================

    def monitor_loop(self):
        navigator = BasicNavigator()
        self.get_logger().info('后台监听中...')

        # 等待电池数据到达
        time.sleep(3.0)
        self.was_charging = self.is_charging
        if self.is_charging:
            self.get_logger().info(
                f'启动时检测到充电状态 '
                f'(电流: {self.current_current:.2f}A)')
            # 已经在充电 → 立即记录充电桩位置
            self.handle_start_charging()
        else:
            self.get_logger().info(
                f'启动时未在充电 (电流: {self.current_current:.2f}A)')

        try:
            while rclpy.ok():
                # --- 职责 A：检测到开始充电 → 记录位置 ---
                if (not self.was_charging and self.is_charging
                        and not self.is_docking):
                    self.handle_start_charging()

                # 更新上一轮状态
                self.was_charging = self.is_charging

                # --- 职责 B：低电量 → 回充 ---
                if (not self.is_docking and not self.is_charging
                        and self.current_battery < self.battery_threshold):
                    self.handle_low_battery(navigator)

                time.sleep(1.0)
        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = AutoDockNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.monitor_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.activate_nav2()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
