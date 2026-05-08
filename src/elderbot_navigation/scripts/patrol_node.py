#!/usr/bin/env python3
"""
巡逻导航 Action Server

功能：
  - 通过 /patrol action 触发自动巡航
  - 支持选择巡逻点、轮次、停留时间和导航超时
  - 支持反馈当前巡逻进度和取消当前巡逻任务
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from elderbot_navigation.action import Patrol


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

        self._goal_lock = threading.Lock()
        self._goal_active = False
        self._callback_group = ReentrantCallbackGroup()

        self._load_waypoint_config()

        self._action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info('==================================================')
        self.get_logger().info('  巡逻 Action Server 已启动')
        self.get_logger().info('  Action 名称: /patrol')
        self.get_logger().info(
            f'  默认巡逻点数量: {len(self.configured_waypoints)}')
        self.get_logger().info(f'  默认停留时间: {self.default_wait_duration}s')
        self.get_logger().info(f'  默认导航超时: {self.default_nav_timeout}s')
        self.get_logger().info('  默认巡逻点列表:')
        for index, waypoint in enumerate(self.configured_waypoints, start=1):
            self.get_logger().info(
                f'    [{index}] {waypoint["name"]}: '
                f'x={waypoint["x"]:.2f}, y={waypoint["y"]:.2f}, '
                f'yaw={waypoint["yaw"]:.2f}')
        self.get_logger().info('==================================================')

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()

    def _load_waypoint_config(self):
        self.declare_parameter('wait_duration', 3.0)
        self.declare_parameter('navigation_timeout', 120.0)
        self.declare_parameter('waypoint_names', ['point_a'])

        self.default_wait_duration = float(
            self.get_parameter('wait_duration').value)
        self.default_nav_timeout = float(
            self.get_parameter('navigation_timeout').value)
        waypoint_names = list(self.get_parameter('waypoint_names').value)

        self.configured_waypoints = []
        self.waypoint_lookup = {}

        for name in waypoint_names:
            self.declare_parameter(f'{name}.x', 0.0)
            self.declare_parameter(f'{name}.y', 0.0)
            self.declare_parameter(f'{name}.yaw', 0.0)

            waypoint = {
                'name': name,
                'x': float(self.get_parameter(f'{name}.x').value),
                'y': float(self.get_parameter(f'{name}.y').value),
                'yaw': float(self.get_parameter(f'{name}.yaw').value),
            }
            self.configured_waypoints.append(waypoint)
            self.waypoint_lookup[name] = waypoint

        if not self.configured_waypoints:
            raise RuntimeError('没有配置巡逻点，无法启动 /patrol action server')

    def goal_callback(self, goal_request: Patrol.Goal):
        try:
            self._resolve_waypoints(goal_request.waypoint_names)
        except ValueError as exc:
            self.get_logger().warn(f'拒绝巡逻目标: {exc}')
            return GoalResponse.REJECT

        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warn('已有巡逻任务正在执行，拒绝新的 goal')
                return GoalResponse.REJECT
            self._goal_active = True

        self.get_logger().info('接收到新的巡逻 goal，请求已接受')
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.get_logger().info('收到巡逻取消请求')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        navigator = None
        result = Patrol.Result()

        try:
            request = goal_handle.request
            waypoints = self._resolve_waypoints(request.waypoint_names)
            wait_duration = (
                float(request.wait_duration)
                if request.override_wait_duration
                else self.default_wait_duration
            )
            navigation_timeout = (
                float(request.navigation_timeout)
                if request.override_navigation_timeout
                else self.default_nav_timeout
            )
            if not request.loop_forever and request.repeat_count > 0:
                target_rounds = int(request.repeat_count)
            else:
                target_rounds = None if request.loop_forever else 1

            navigator = BasicNavigator()
            self.get_logger().info('等待 Navigation2 激活...')
            navigator.waitUntilNav2Active()

            total_waypoints = len(waypoints)
            success_count = 0
            failed_count = 0
            timed_out_count = 0
            rounds_completed = 0

            self.get_logger().info(
                '开始执行巡逻: '
                f'waypoints={[wp["name"] for wp in waypoints]}, '
                f'rounds={"infinite" if target_rounds is None else target_rounds}, '
                f'wait_duration={wait_duration}, '
                f'navigation_timeout={navigation_timeout}')

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    return self._cancel_goal(
                        goal_handle, navigator, result, rounds_completed,
                        success_count, failed_count, timed_out_count,
                        '巡逻任务在新一轮开始前被取消')

                if target_rounds is not None and rounds_completed >= target_rounds:
                    break

                current_round = rounds_completed + 1
                self.get_logger().info(f'>>> 第 {current_round} 轮巡逻开始 <<<')

                for waypoint_index, waypoint in enumerate(waypoints, start=1):
                    if goal_handle.is_cancel_requested:
                        return self._cancel_goal(
                            goal_handle, navigator, result, rounds_completed,
                            success_count, failed_count, timed_out_count,
                            '巡逻任务在导航过程中被取消')

                    self._publish_feedback(
                        goal_handle,
                        current_round=current_round,
                        waypoint_index=waypoint_index,
                        total_waypoints=total_waypoints,
                        waypoint_name=waypoint['name'],
                        state='navigating',
                        elapsed_time=0.0,
                    )

                    self.get_logger().info(
                        f'[{waypoint_index}/{total_waypoints}] '
                        f'正在导航到 {waypoint["name"]} '
                        f'(x={waypoint["x"]:.2f}, y={waypoint["y"]:.2f})')
                    navigator.goToPose(self.create_goal_pose(waypoint))

                    start_time = time.monotonic()
                    timed_out = False
                    while not navigator.isTaskComplete():
                        if goal_handle.is_cancel_requested:
                            return self._cancel_goal(
                                goal_handle, navigator, result, rounds_completed,
                                success_count, failed_count, timed_out_count,
                                '巡逻任务在导航过程中被取消')

                        elapsed = time.monotonic() - start_time
                        self._publish_feedback(
                            goal_handle,
                            current_round=current_round,
                            waypoint_index=waypoint_index,
                            total_waypoints=total_waypoints,
                            waypoint_name=waypoint['name'],
                            state='navigating',
                            elapsed_time=elapsed,
                        )

                        if navigation_timeout > 0.0 and elapsed > navigation_timeout:
                            timed_out = True
                            self.get_logger().warn(
                                f'导航到 {waypoint["name"]} 超时，取消当前导航并跳过')
                            navigator.cancelTask()
                            self._wait_for_task_stop(navigator)
                            break

                        time.sleep(0.1)

                    if timed_out:
                        timed_out_count += 1
                        self._publish_feedback(
                            goal_handle,
                            current_round=current_round,
                            waypoint_index=waypoint_index,
                            total_waypoints=total_waypoints,
                            waypoint_name=waypoint['name'],
                            state='timed_out',
                            elapsed_time=time.monotonic() - start_time,
                        )
                        continue

                    nav_result = navigator.getResult()
                    if nav_result == TaskResult.SUCCEEDED:
                        success_count += 1
                        self.get_logger().info(f'成功到达 {waypoint["name"]}')
                        self._publish_feedback(
                            goal_handle,
                            current_round=current_round,
                            waypoint_index=waypoint_index,
                            total_waypoints=total_waypoints,
                            waypoint_name=waypoint['name'],
                            state='arrived',
                            elapsed_time=time.monotonic() - start_time,
                        )
                        if wait_duration > 0.0:
                            self.get_logger().info(
                                f'在 {waypoint["name"]} 停留 {wait_duration}s')
                            completed_wait = self._wait_at_waypoint(
                                goal_handle=goal_handle,
                                wait_duration=wait_duration,
                                current_round=current_round,
                                waypoint_index=waypoint_index,
                                total_waypoints=total_waypoints,
                                waypoint_name=waypoint['name'],
                            )
                            if not completed_wait:
                                return self._cancel_goal(
                                    goal_handle, navigator, result, rounds_completed,
                                    success_count, failed_count, timed_out_count,
                                    '巡逻任务在停留等待时被取消')
                    elif nav_result == TaskResult.CANCELED:
                        if goal_handle.is_cancel_requested:
                            return self._cancel_goal(
                                goal_handle, navigator, result, rounds_completed,
                                success_count, failed_count, timed_out_count,
                                '巡逻任务被取消')

                        failed_count += 1
                        self.get_logger().warn(
                            f'导航到 {waypoint["name"]} 被外部取消，跳过该点')
                        self._publish_feedback(
                            goal_handle,
                            current_round=current_round,
                            waypoint_index=waypoint_index,
                            total_waypoints=total_waypoints,
                            waypoint_name=waypoint['name'],
                            state='canceled',
                            elapsed_time=time.monotonic() - start_time,
                        )
                    else:
                        failed_count += 1
                        self.get_logger().warn(
                            f'导航到 {waypoint["name"]} 失败，跳过该点')
                        self._publish_feedback(
                            goal_handle,
                            current_round=current_round,
                            waypoint_index=waypoint_index,
                            total_waypoints=total_waypoints,
                            waypoint_name=waypoint['name'],
                            state='failed',
                            elapsed_time=time.monotonic() - start_time,
                        )

                rounds_completed += 1
                self.get_logger().info(f'>>> 第 {current_round} 轮巡逻完成 <<<')

            result.success = True
            result.rounds_completed = rounds_completed
            result.waypoints_succeeded = success_count
            result.waypoints_failed = failed_count
            result.waypoints_timed_out = timed_out_count
            result.message = (
                f'巡逻完成: rounds={rounds_completed}, '
                f'succeeded={success_count}, failed={failed_count}, '
                f'timed_out={timed_out_count}'
            )
            goal_handle.succeed()
            self.get_logger().info(result.message)
            return result

        except Exception as exc:
            if navigator is not None:
                try:
                    navigator.cancelTask()
                except Exception:
                    pass

            result.success = False
            result.message = f'巡逻执行异常: {exc}'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        finally:
            with self._goal_lock:
                self._goal_active = False

    def _resolve_waypoints(self, waypoint_names):
        if not waypoint_names:
            return list(self.configured_waypoints)

        waypoints = []
        missing_names = []
        for name in waypoint_names:
            waypoint = self.waypoint_lookup.get(name)
            if waypoint is None:
                missing_names.append(name)
            else:
                waypoints.append(waypoint)

        if missing_names:
            raise ValueError(f'未找到巡逻点: {missing_names}')

        return waypoints

    def create_goal_pose(self, waypoint: dict) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = waypoint['x']
        goal.pose.position.y = waypoint['y']
        goal.pose.position.z = 0.0
        goal.pose.orientation = euler_to_quaternion(waypoint['yaw'])
        return goal

    def _publish_feedback(
        self,
        goal_handle,
        current_round,
        waypoint_index,
        total_waypoints,
        waypoint_name,
        state,
        elapsed_time,
    ):
        feedback = Patrol.Feedback()
        feedback.current_round = int(current_round)
        feedback.current_waypoint_index = int(waypoint_index)
        feedback.total_waypoints = int(total_waypoints)
        feedback.current_waypoint_name = waypoint_name
        feedback.state = state
        feedback.elapsed_time = float(elapsed_time)
        goal_handle.publish_feedback(feedback)

    def _wait_at_waypoint(
        self,
        goal_handle,
        wait_duration,
        current_round,
        waypoint_index,
        total_waypoints,
        waypoint_name,
    ):
        deadline = time.monotonic() + wait_duration
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False

            elapsed = wait_duration - max(0.0, deadline - time.monotonic())
            self._publish_feedback(
                goal_handle,
                current_round=current_round,
                waypoint_index=waypoint_index,
                total_waypoints=total_waypoints,
                waypoint_name=waypoint_name,
                state='waiting',
                elapsed_time=elapsed,
            )
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

        return rclpy.ok()

    def _wait_for_task_stop(self, navigator, timeout_sec=2.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if navigator.isTaskComplete():
                return
            time.sleep(0.05)

    def _cancel_goal(
        self,
        goal_handle,
        navigator,
        result,
        rounds_completed,
        success_count,
        failed_count,
        timed_out_count,
        message,
    ):
        if navigator is not None:
            try:
                navigator.cancelTask()
                self._wait_for_task_stop(navigator)
            except Exception:
                pass

        result.success = False
        result.rounds_completed = rounds_completed
        result.waypoints_succeeded = success_count
        result.waypoints_failed = failed_count
        result.waypoints_timed_out = timed_out_count
        result.message = message
        goal_handle.canceled()
        self.get_logger().info(message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，巡逻 Action Server 退出')
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
