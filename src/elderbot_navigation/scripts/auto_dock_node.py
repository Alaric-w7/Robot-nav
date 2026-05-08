#!/usr/bin/env python3
"""
自动回充监控 + Action Server 节点

功能：
  - 检测到开始充电时，通过 TF2 记录充电桩位置和回充准备点
  - 电量低于阈值时自动执行回充
  - 通过 /auto_dock action 强制执行回充，不受电量阈值限制
"""

import math
import os
import subprocess
import threading
import time

import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformException

from elderbot_navigation.action import AutoDock


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
    cmd = (
        f"bash -c 'source /opt/ros/humble/setup.bash && "
        f"ros2 lifecycle set /{node_name} {transition}'"
    )
    try:
        subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=8
        )
    except Exception:
        pass


class AutoDockNode(Node):
    def __init__(self):
        super().__init__('auto_dock_node')

        self._dock_lock = threading.Lock()
        self._action_goal_lock = threading.Lock()
        self._action_goal_active = False
        self._action_callback_group = ReentrantCallbackGroup()

        # ==================== 参数 ====================
        self.declare_parameter('battery_threshold', 0.20)
        self.declare_parameter('backup_speed', 0.3)
        self.declare_parameter('backup_max_dist', 0.35)
        self.declare_parameter('prep_distance', 0.30)
        self.declare_parameter('timeout', 30.0)
        self.declare_parameter('default_charger_x', -0.77)
        self.declare_parameter('default_charger_y', 1.17)
        self.declare_parameter('default_charger_yaw', 2.36)

        self.battery_threshold = float(
            self.get_parameter('battery_threshold').value
        )
        self.backup_speed = float(self.get_parameter('backup_speed').value)
        self.backup_max_dist = float(
            self.get_parameter('backup_max_dist').value
        )
        self.prep_distance = float(self.get_parameter('prep_distance').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.default_charger_x = float(
            self.get_parameter('default_charger_x').value
        )
        self.default_charger_y = float(
            self.get_parameter('default_charger_y').value
        )
        self.default_charger_yaw = float(
            self.get_parameter('default_charger_yaw').value
        )

        # dock_pose.yaml 路径
        self.dock_pose_file = os.path.expanduser(
            '~/elderbot_ws/src/elderbot_navigation/config/dock_pose.yaml'
        )

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
            BatteryState, '/battery_state', self.battery_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ==================== Action ====================
        self._action_server = ActionServer(
            self,
            AutoDock,
            'auto_dock',
            execute_callback=self.execute_auto_dock_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_callback_group,
        )

        # 确保 dock_pose.yaml 存在
        if not os.path.exists(self.dock_pose_file):
            self._create_default_dock_pose()

        dock = self.load_dock_pose()
        self.get_logger().info('=' * 50)
        self.get_logger().info('  自动回充监控 + Action Server')
        self.get_logger().info(
            f'  低电量阈值: <{self.battery_threshold*100:.0f}%'
        )
        self.get_logger().info('  Action 名称: /auto_dock')
        if dock:
            self.get_logger().info(
                f'  充电桩位置: ({dock["charger_x"]}, '
                f'{dock["charger_y"]}) yaw={dock["charger_yaw"]}'
            )
            self.get_logger().info(
                f'  回充准备点: ({dock["prep_x"]}, '
                f'{dock["prep_y"]}) yaw={dock["prep_yaw"]}'
            )
        self.get_logger().info(
            f'  倒车: 最大{self.backup_max_dist*100:.0f}cm '
            f'@ {self.backup_speed}m/s'
        )
        self.get_logger().info(f'  配置文件: {self.dock_pose_file}')
        self.get_logger().info('=' * 50)

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()

    # ==================== 回调 ====================

    def battery_callback(self, msg: BatteryState):
        val = msg.percentage
        if val > 1.0:
            val = val / 100.0
        self.current_battery = float(val)
        self.current_current = float(msg.current)
        self.is_charging = msg.current > 0.1

    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    # ==================== Action Server ====================

    def goal_callback(self, _goal_request: AutoDock.Goal):
        with self._action_goal_lock:
            if self._action_goal_active or self.is_docking:
                self.get_logger().warn('已有回充任务正在执行，拒绝新的 /auto_dock goal')
                return GoalResponse.REJECT
            self._action_goal_active = True

        self.get_logger().info('接收到 /auto_dock goal，请求已接受')
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.get_logger().info('收到 /auto_dock 取消请求')
        return CancelResponse.ACCEPT

    def execute_auto_dock_callback(self, goal_handle):
        request_source = goal_handle.request.request_source.strip()
        trigger_reason = (
            f'action:{request_source}'
            if request_source else 'action:manual_request'
        )

        try:
            state, charged, message = self.run_docking_sequence(
                trigger_reason=trigger_reason,
                force_dock=True,
                goal_handle=goal_handle,
            )

            result = self._build_action_result(
                success=(state == 'succeeded'),
                charged=charged,
                message=message,
            )

            if state == 'succeeded':
                goal_handle.succeed()
            elif state == 'canceled':
                goal_handle.canceled()
            else:
                goal_handle.abort()

            return result
        finally:
            with self._action_goal_lock:
                self._action_goal_active = False

    def _publish_action_feedback(self, goal_handle, stage, message):
        if goal_handle is None:
            return

        feedback = AutoDock.Feedback()
        feedback.stage = stage
        feedback.is_charging = self.is_charging
        feedback.battery_percent = float(self.current_battery * 100.0)
        feedback.charging_current = float(self.current_current)
        feedback.message = message
        goal_handle.publish_feedback(feedback)

    def _build_action_result(self, success, charged, message):
        result = AutoDock.Result()
        result.success = bool(success)
        result.charged = bool(charged)
        result.battery_percent = float(self.current_battery * 100.0)
        result.charging_current = float(self.current_current)
        result.message = message
        return result

    def _sleep_with_cancel(
        self,
        duration_sec,
        goal_handle=None,
        stage='waiting',
        message='',
        interval_sec=0.1,
    ):
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return False

            self._publish_action_feedback(goal_handle, stage, message)
            time.sleep(min(interval_sec, deadline - time.monotonic()))

        return rclpy.ok()

    # ==================== 工具方法 ====================

    def get_map_pose(self):
        """通过 TF2 获取机器人在 map 坐标系下的位姿"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            yaw = yaw_from_quaternion(transform.transform.rotation)
            return x, y, yaw
        except TransformException as exc:
            self.get_logger().warn(f'TF 查询失败: {exc}')
            return None

    def _create_default_dock_pose(self):
        """用默认参数创建初始 dock_pose.yaml"""
        charger_x = self.default_charger_x
        charger_y = self.default_charger_y
        charger_yaw = self.default_charger_yaw
        prep_x = charger_x + self.prep_distance * math.cos(charger_yaw)
        prep_y = charger_y + self.prep_distance * math.sin(charger_yaw)
        self.save_dock_pose(
            charger_x,
            charger_y,
            charger_yaw,
            prep_x,
            prep_y,
            charger_yaw,
        )
        self.get_logger().info(
            f'首次启动，已创建默认 dock_pose: '
            f'charger=({charger_x}, {charger_y}), '
            f'prep=({prep_x:.4f}, {prep_y:.4f})'
        )

    def save_dock_pose(self, charger_x, charger_y, charger_yaw, prep_x, prep_y, prep_yaw):
        """保存充电桩位置 + 回充准备点到 yaml"""
        data = {
            'charger_x': round(float(charger_x), 4),
            'charger_y': round(float(charger_y), 4),
            'charger_yaw': round(float(charger_yaw), 4),
            'prep_x': round(float(prep_x), 4),
            'prep_y': round(float(prep_y), 4),
            'prep_yaw': round(float(prep_yaw), 4),
        }
        os.makedirs(os.path.dirname(self.dock_pose_file), exist_ok=True)
        with open(self.dock_pose_file, 'w', encoding='utf-8') as file_obj:
            file_obj.write('# 自动回充配置 - 充电时自动记录\n')
            file_obj.write(f'# 最后更新: {time.strftime("%Y-%m-%d %H:%M:%S")}\n\n')
            file_obj.write('# 充电桩位置（机器人在充电桩上的坐标）\n')
            file_obj.write(f'charger_x: {data["charger_x"]}\n')
            file_obj.write(f'charger_y: {data["charger_y"]}\n')
            file_obj.write(f'charger_yaw: {data["charger_yaw"]}\n\n')
            file_obj.write('# 回充准备点（充电桩前方30cm，导航目标）\n')
            file_obj.write(f'prep_x: {data["prep_x"]}\n')
            file_obj.write(f'prep_y: {data["prep_y"]}\n')
            file_obj.write(f'prep_yaw: {data["prep_yaw"]}\n')

        self.get_logger().info(
            f'   已保存: 充电桩({charger_x:.4f}, {charger_y:.4f}, '
            f'yaw={charger_yaw:.4f}) 准备点({prep_x:.4f}, '
            f'{prep_y:.4f}, yaw={prep_yaw:.4f})'
        )

    def load_dock_pose(self):
        """读取 dock_pose.yaml"""
        try:
            with open(self.dock_pose_file, 'r', encoding='utf-8') as file_obj:
                return yaml.safe_load(file_obj)
        except Exception as exc:
            self.get_logger().warn(f'读取 dock_pose 失败: {exc}')
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
        if rclpy.ok():
            self.get_logger().info('   [Nav2] 去激活 controller + smoother...')
        lifecycle_set_state('controller_server', 'deactivate')
        lifecycle_set_state('velocity_smoother', 'deactivate')
        time.sleep(0.5)
        if rclpy.ok():
            self.get_logger().info('   [Nav2] 已去激活 ✓')

    def activate_nav2(self):
        if rclpy.ok():
            self.get_logger().info('   [Nav2] 恢复 controller + smoother...')
        lifecycle_set_state('velocity_smoother', 'activate')
        lifecycle_set_state('controller_server', 'activate')
        time.sleep(0.5)
        if rclpy.ok():
            self.get_logger().info('   [Nav2] 已恢复 ✓')

    # ==================== 回充动作 ====================

    def backup_until_charged(self, max_dist, speed, goal_handle=None):
        """里程计闭环倒车，检测到充电立刻停"""
        start_x = self.odom_x
        start_y = self.odom_y
        start_time = time.monotonic()
        charged = False
        canceled = False

        twist = Twist()
        twist.linear.x = -speed

        while rclpy.ok():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                canceled = True
                self.get_logger().info('   收到取消请求，停止倒车对接')
                break

            dist = math.sqrt(
                (self.odom_x - start_x) ** 2 +
                (self.odom_y - start_y) ** 2
            )
            elapsed = time.monotonic() - start_time

            self._publish_action_feedback(
                goal_handle,
                stage='backing_up',
                message=(
                    f'正在倒车对接，已退后 {dist*100:.1f}cm，'
                    f'已耗时 {elapsed:.1f}s'
                ),
            )

            if self.current_current > 0.1:
                charged = True
                self.get_logger().info(
                    f'   ⚡ 检测到充电! (电流: {self.current_current:.2f}A) '
                    f'立刻停车'
                )
                break

            if dist >= max_dist:
                self.get_logger().warn(
                    f'   已倒车 {dist*100:.1f}cm (最大 '
                    f'{max_dist*100:.0f}cm)，未检测到充电'
                )
                break

            if elapsed > self.timeout:
                self.get_logger().warn(f'   倒车超时 ({self.timeout}s)')
                break

            self.cmd_pub.publish(twist)
            time.sleep(0.02)

        self.full_stop()

        final_dist = math.sqrt(
            (self.odom_x - start_x) ** 2 +
            (self.odom_y - start_y) ** 2
        )
        elapsed = time.monotonic() - start_time
        status = '✓ 已充电' if charged else '✗ 未充电'
        self.get_logger().info(
            f'   倒车完成 {status} 实际: {final_dist*100:.1f}cm '
            f'用时: {elapsed:.1f}s'
        )
        return charged, canceled

    def run_docking_sequence(self, trigger_reason, force_dock=False, goal_handle=None):
        with self._dock_lock:
            if self.is_docking:
                message = '已有回充流程正在执行'
                self.get_logger().warn(message)
                return 'failed', self.is_charging, message
            self.is_docking = True

        navigator = None
        nav2_deactivated = False

        try:
            battery_percent = self.current_battery * 100.0
            if self.is_charging:
                message = '机器人当前已在充电，无需再次执行回充'
                self.get_logger().info(message)
                self._publish_action_feedback(
                    goal_handle, 'already_charging', message
                )
                return 'succeeded', True, message

            if force_dock:
                self.get_logger().warn(
                    f'!!! 收到强制回充请求 ({trigger_reason})，'
                    f'当前电量 {battery_percent:.1f}% !!!'
                )
            else:
                self.get_logger().warn(
                    f'!!! 低电量 ({battery_percent:.1f}%) -> 启动回充 !!!'
                )

            dock = self.load_dock_pose()
            if dock is None:
                message = 'dock_pose.yaml 不存在或无法读取，回充取消'
                self.get_logger().error(message)
                return 'failed', False, message

            prep_x = dock['prep_x']
            prep_y = dock['prep_y']
            prep_yaw = dock['prep_yaw']

            navigator = BasicNavigator()
            self._publish_action_feedback(
                goal_handle, 'waiting_nav2', '等待 Navigation2 激活'
            )
            self.get_logger().info('-> 等待 Navigation2 激活...')
            navigator.waitUntilNav2Active()

            if goal_handle is not None and goal_handle.is_cancel_requested:
                message = '回充任务在等待 Navigation2 时被取消'
                self.get_logger().info(message)
                return 'canceled', False, message

            self.get_logger().info('-> 取消当前导航任务...')
            navigator.cancelTask()
            while rclpy.ok() and not navigator.isTaskComplete():
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    message = '回充任务在取消当前导航时被取消'
                    self.get_logger().info(message)
                    return 'canceled', False, message
                time.sleep(0.1)

            self.get_logger().info(
                f'-> 1. 导航到回充准备点 '
                f'({prep_x}, {prep_y}, yaw={prep_yaw})...'
            )
            self._publish_action_feedback(
                goal_handle,
                'navigating_to_prep',
                f'导航到回充准备点 ({prep_x:.3f}, {prep_y:.3f})',
            )
            pre_pose = self.create_goal_pose(prep_x, prep_y, prep_yaw)
            navigator.goToPose(pre_pose)

            while rclpy.ok() and not navigator.isTaskComplete():
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    navigator.cancelTask()
                    while rclpy.ok() and not navigator.isTaskComplete():
                        time.sleep(0.05)
                    message = '回充任务在前往准备点时被取消'
                    self.get_logger().info(message)
                    return 'canceled', False, message

                self._publish_action_feedback(
                    goal_handle,
                    'navigating_to_prep',
                    '正在前往回充准备点',
                )
                time.sleep(0.5)

            nav_result = navigator.getResult()
            if nav_result != TaskResult.SUCCEEDED:
                message = '准备点不可达，回充取消'
                self.get_logger().error(message)
                return 'failed', False, message

            self.get_logger().info('-> 到达准备点 ✓，等 2 秒刹车...')
            if not self._sleep_with_cancel(
                2.0,
                goal_handle=goal_handle,
                stage='stabilizing',
                message='已到达准备点，等待底盘稳定',
            ):
                message = '回充任务在准备点等待时被取消'
                self.get_logger().info(message)
                return 'canceled', False, message

            self._publish_action_feedback(
                goal_handle,
                'deactivating_nav2',
                '去激活 Nav2 controller 和 smoother',
            )
            self.deactivate_nav2()
            nav2_deactivated = True

            self.get_logger().info(
                f'-> 2. 倒车最多 {self.backup_max_dist*100:.0f}cm '
                f'(检测充电电流)...'
            )
            charged, canceled = self.backup_until_charged(
                self.backup_max_dist,
                self.backup_speed,
                goal_handle=goal_handle,
            )

            if canceled:
                message = '回充任务在倒车对接时被取消'
                self.get_logger().info(message)
                return 'canceled', charged, message

            if charged:
                message = '对接成功，已接通充电'
                self.get_logger().info(f'-> ⚡ {message} ⚡')
                self._publish_action_feedback(
                    goal_handle, 'finished', message
                )
                return 'succeeded', True, message

            message = '对接未成功，未检测到充电电流'
            self.get_logger().warn(f'-> {message}')
            self._publish_action_feedback(goal_handle, 'finished', message)
            return 'failed', False, message

        except Exception as exc:
            message = f'回充执行异常: {exc}'
            self.get_logger().error(message)
            self._publish_action_feedback(goal_handle, 'error', message)
            return 'failed', self.is_charging, message

        finally:
            if nav2_deactivated:
                self.activate_nav2()
            self.get_logger().info('-> 回充流程结束')
            self.is_docking = False

    # ==================== 职责 A：检测充电 -> 记录位置 ====================

    def handle_start_charging(self):
        """检测到开始充电 -> 记录充电桩位置 + 计算回充准备点"""
        self.get_logger().info('=' * 40)
        self.get_logger().info('>>> 检测到充电，记录充电桩位置 <<<')

        self.get_logger().info('-> 等待 3 秒让定位稳定...')
        time.sleep(3.0)

        if not self.is_charging:
            self.get_logger().info('充电已断开，取消记录（可能是瞬间接触）')
            return

        pose = self.get_map_pose()
        if pose is None:
            self.get_logger().error('   TF 查询失败，无法记录位置')
            return

        charger_x, charger_y, charger_yaw = pose
        self.get_logger().info(
            f'-> 充电桩位置: x={charger_x:.4f}, y={charger_y:.4f}, '
            f'yaw={charger_yaw:.4f}'
        )

        prep_x = charger_x + self.prep_distance * math.cos(charger_yaw)
        prep_y = charger_y + self.prep_distance * math.sin(charger_yaw)
        prep_yaw = charger_yaw

        self.get_logger().info(
            f'-> 回充准备点: x={prep_x:.4f}, y={prep_y:.4f}, '
            f'yaw={prep_yaw:.4f}'
        )

        self.save_dock_pose(
            charger_x, charger_y, charger_yaw, prep_x, prep_y, prep_yaw
        )

        self.get_logger().info('>>> 位置记录完成（零打扰，继续充电）<<<')
        self.get_logger().info('=' * 40)

    # ==================== 职责 B：低电量 -> 回充 ====================

    def handle_low_battery(self):
        self.run_docking_sequence(
            trigger_reason='monitor:low_battery',
            force_dock=False,
            goal_handle=None,
        )

    # ==================== 主循环 ====================

    def monitor_loop(self):
        self.get_logger().info('后台监听中...')

        time.sleep(3.0)
        self.was_charging = self.is_charging
        if self.is_charging:
            self.get_logger().info(
                f'启动时检测到充电状态 '
                f'(电流: {self.current_current:.2f}A)'
            )
            self.handle_start_charging()
        else:
            self.get_logger().info(
                f'启动时未在充电 (电流: {self.current_current:.2f}A)'
            )

        try:
            while rclpy.ok():
                if (not self.was_charging and self.is_charging
                        and not self.is_docking):
                    self.handle_start_charging()

                self.was_charging = self.is_charging

                if (not self.is_docking and not self._action_goal_active
                        and not self.is_charging
                        and self.current_battery < self.battery_threshold):
                    self.handle_low_battery()

                time.sleep(1.0)
        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = AutoDockNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.monitor_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.activate_nav2()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
