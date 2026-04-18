import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class DepthScanFilter(Node):
    def __init__(self):
        super().__init__('depth_scan_filter')
        self.declare_parameter('input_topic', '/scan_depth_raw')
        self.declare_parameter('output_topic', '/scan_depth')
        self.declare_parameter('range_max_override', 6.0)
        self.declare_parameter('clear_padding', 0.1)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.range_max_override = float(self.get_parameter('range_max_override').value)
        self.clear_padding = float(self.get_parameter('clear_padding').value)

        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.get_logger().info(f'Filtering depth scan: {input_topic} -> {output_topic}')

    def listener_callback(self, msg):
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = max(msg.range_max, self.range_max_override)
        new_msg.intensities = list(msg.intensities)

        clear_range = min(msg.range_max + self.clear_padding, new_msg.range_max)
        new_ranges = []
        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r):
                # 用略大于 obstacle_max_range 的射线做清障，但仍保持在消息 range_max 内。
                new_ranges.append(clear_range)
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
