import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class DepthScanFilter(Node):
    def __init__(self):
        super().__init__('depth_scan_filter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_depth_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, '/scan_depth', 10)

    def listener_callback(self, msg):
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = 6.0  # 必须改写系统消息体上限，否则后方雷达库会丢弃大于5.0的点，导致永远无法清除
        new_msg.intensities = msg.intensities
        
        new_ranges = []
        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r):
                # 聪明的绝招：设为 5.1 米。
                # 5.1m > obstacle_max_range (5.0m)，所以绝对不会被划分为障碍物（解决巨大圆圈问题）
                # 5.1m < raytrace_max_range (6.0m)，所以能够发出 5.1m 的射线去清除范围内的旧障碍物！
                new_ranges.append(msg.range_max + 0.1)
            else:
                new_ranges.append(r)
        
        new_msg.ranges = new_ranges
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    depth_scan_filter = DepthScanFilter()
    rclpy.spin(depth_scan_filter)
    depth_scan_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
