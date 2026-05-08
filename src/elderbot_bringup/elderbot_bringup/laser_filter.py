import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserFilter(Node):
    def __init__(self):
        super().__init__('laser_filter')

        self.declare_parameter('range_min_filter', 0.20)
        self.range_min = self.get_parameter('range_min_filter').value

        # 后方盲区角度 (度): 过滤掉后方多少度
        # 默认 90.0 度 => 过滤后方 90度, 保留前方 270度
        # 设为 0.0 则不过滤 (保留全部360度)
        #
        # rplidar C1 驱动的角度约定:
        #   angle_min ≈ -π, angle_max ≈ +π
        #   angle 0   = 小车正后方 (雷达物理0°经 M_PI-angle 变换)
        #   angle ±π  = 小车正前方
        # 因此 "后方90°" 指的是 angle 在 [-45°, +45°] 范围内的点
        self.declare_parameter('rear_blind_angle', 90.0)
        rear_blind_deg = self.get_parameter('rear_blind_angle').value
        self.rear_blind_half = np.deg2rad(rear_blind_deg / 2.0)

        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(LaserScan, 'scan_filtered', 10)

        self.get_logger().info(
            f'Laser Filter: range_min={self.range_min:.2f}m, '
            f'rear_blind={rear_blind_deg:.0f}° '
            f'(keep front {360.0 - rear_blind_deg:.0f}°)')

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)

        # 最小距离过滤
        ranges[ranges < self.range_min] = np.inf

        # 角度过滤:
        # 归一化到 [-π, π], 其中 0 = 后方, ±π = 前方
        raw_angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        angles = np.arctan2(np.sin(raw_angles), np.cos(raw_angles))

        # 后方盲区: |angle| < rear_blind_half 的点在后方, 设为 inf
        mask_rear = np.abs(angles) < self.rear_blind_half
        ranges[mask_rear] = np.inf

        self.get_logger().info(
            f'angle_min={np.rad2deg(msg.angle_min):.1f}° '
            f'angle_max={np.rad2deg(msg.angle_max):.1f}° '
            f'filtered={np.sum(mask_rear)}/{len(ranges)} points',
            throttle_duration_sec=5.0)

        msg.ranges = ranges.tolist()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
