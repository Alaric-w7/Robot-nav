import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('delay_sec', 4.0)
        self.declare_parameter('repeat_count', 5)

        self.initial_x = float(self.get_parameter('x').value)
        self.initial_y = float(self.get_parameter('y').value)
        self.initial_yaw = float(self.get_parameter('yaw').value)
        self.repeat_count = int(self.get_parameter('repeat_count').value)
        delay_sec = float(self.get_parameter('delay_sec').value)

        self.publish_times = 0
        self.startup_timer = self.create_timer(delay_sec, self._start_publishing)
        self.publish_timer = None

    def _start_publishing(self):
        self.startup_timer.cancel()
        self.publish_timer = self.create_timer(0.5, self.publish_initial_pose)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # 坐标系通常为 map

        # 设置初始位置 (x, y, z)
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.position.z = 0.0

        # 设置初始姿态 (四元数 x, y, z, w)
        half_yaw = self.initial_yaw * 0.5
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        # 设置协方差 (Covariance)
        # AMCL 等定位算法通常需要协方差矩阵
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]

        self.publisher_.publish(msg)
        self.publish_times += 1
        self.get_logger().info(
            f'已发布初始位姿({self.publish_times}/{self.repeat_count}): '
            f'x={self.initial_x:.2f}, y={self.initial_y:.2f}, yaw={self.initial_yaw:.2f}, frame_id=map'
        )

        # 发布完成后取消定时器，避免重复发布
        if self.publish_times >= self.repeat_count and self.publish_timer is not None:
            self.publish_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
