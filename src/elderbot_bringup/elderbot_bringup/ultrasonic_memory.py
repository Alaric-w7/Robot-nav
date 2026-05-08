import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener


class UltrasonicMemory(Node):
    def __init__(self):
        super().__init__('ultrasonic_memory')

        self.declare_parameter('scan_topic', '/scan_ultrasonic')
        self.declare_parameter('cloud_topic', '/ultrasonic_memory_cloud')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('memory_duration', 180.0)
        self.declare_parameter('mark_min_range', 0.05)
        self.declare_parameter('mark_max_range', 1.0)
        self.declare_parameter('grid_resolution', 0.08)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('min_confirm_age', 0.0)
        self.declare_parameter('clear_enabled', True)
        self.declare_parameter('clear_max_range', 1.2)
        self.declare_parameter('clear_margin', 0.12)
        self.declare_parameter('clear_hits', 5)
        self.declare_parameter('clear_step', 0.04)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('point_z', 0.25)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.memory_duration = float(self.get_parameter('memory_duration').value)
        self.mark_min_range = float(self.get_parameter('mark_min_range').value)
        self.mark_max_range = float(self.get_parameter('mark_max_range').value)
        self.grid_resolution = float(self.get_parameter('grid_resolution').value)
        self.min_hits = int(self.get_parameter('min_hits').value)
        self.min_confirm_age = float(self.get_parameter('min_confirm_age').value)
        self.clear_enabled = bool(self.get_parameter('clear_enabled').value)
        self.clear_max_range = float(self.get_parameter('clear_max_range').value)
        self.clear_margin = float(self.get_parameter('clear_margin').value)
        self.clear_hits = max(1, int(self.get_parameter('clear_hits').value))
        self.clear_step = max(
            0.01,
            float(self.get_parameter('clear_step').value),
        )
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.point_z = float(self.get_parameter('point_z').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cells = {}
        self.cloud_pub = self.create_publisher(PointCloud2, self.cloud_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_service(Empty, '/clear_ultrasonic_memory', self.clear_callback)

        timer_period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.5
        self.create_timer(timer_period, self.publish_cloud)

        self.get_logger().info(
            f'Ultrasonic memory started: {self.scan_topic} -> {self.cloud_topic}, '
            f'target_frame={self.target_frame}, duration={self.memory_duration}s, '
            f'max_range={self.mark_max_range}m, min_hits={self.min_hits}, '
            f'min_confirm_age={self.min_confirm_age}s, '
            f'clear_enabled={self.clear_enabled}, clear_hits={self.clear_hits}'
        )

    @staticmethod
    def rotate_point(q, x, y, z):
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)

        rx = x + qw * tx + (qy * tz - qz * ty)
        ry = y + qw * ty + (qz * tx - qx * tz)
        rz = z + qw * tz + (qx * ty - qy * tx)
        return rx, ry, rz

    def transform_point(self, transform, x, y, z):
        rx, ry, rz = self.rotate_point(transform.rotation, x, y, z)
        return (
            rx + transform.translation.x,
            ry + transform.translation.y,
            rz + transform.translation.z,
        )

    def key_for_point(self, x, y):
        return (
            int(round(x / self.grid_resolution)),
            int(round(y / self.grid_resolution)),
        )

    def add_clear_ray_keys(self, transform, angle, clear_limit, clear_keys):
        distance = self.mark_min_range
        while distance <= clear_limit:
            x_base = distance * math.cos(angle)
            y_base = distance * math.sin(angle)
            x_map, y_map, _ = self.transform_point(
                transform,
                x_base,
                y_base,
                self.point_z,
            )
            clear_keys.add(self.key_for_point(x_map, y_map))
            distance += self.clear_step

    def clear_memory_cells(self, clear_keys, updated_keys):
        for key in clear_keys:
            if key in updated_keys:
                continue
            cell = self.cells.get(key)
            if cell is None:
                continue
            cell['clear_hits'] = cell.get('clear_hits', 0) + 1
            if cell['clear_hits'] >= self.clear_hits:
                del self.cells[key]

    def scan_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
            ).transform
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot transform {msg.header.frame_id} to {self.target_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        now = time.monotonic()
        updated_keys = set()
        clear_keys = set()
        angle = msg.angle_min

        for distance in msg.ranges:
            if math.isfinite(distance) and self.mark_min_range <= distance <= self.mark_max_range:
                x_base = distance * math.cos(angle)
                y_base = distance * math.sin(angle)
                x_map, y_map, z_map = self.transform_point(
                    transform,
                    x_base,
                    y_base,
                    self.point_z,
                )
                key = self.key_for_point(x_map, y_map)
                if key not in updated_keys:
                    cell = self.cells.get(key)
                    if cell is None:
                        cell = {
                            'x': x_map,
                            'y': y_map,
                            'z': z_map,
                            'hits': 0,
                            'first_seen': now,
                        }
                        self.cells[key] = cell
                    cell['x'] = x_map
                    cell['y'] = y_map
                    cell['z'] = z_map
                    cell['hits'] += 1
                    cell['clear_hits'] = 0
                    cell['last_seen'] = now
                    updated_keys.add(key)
            if self.clear_enabled and not math.isnan(distance):
                if math.isfinite(distance):
                    clear_limit = min(
                        distance - self.clear_margin,
                        self.clear_max_range,
                    )
                else:
                    clear_limit = self.clear_max_range
                if clear_limit >= self.mark_min_range:
                    self.add_clear_ray_keys(
                        transform,
                        angle,
                        clear_limit,
                        clear_keys,
                    )
            angle += msg.angle_increment

        self.clear_memory_cells(clear_keys, updated_keys)
        self.prune(now)

    def prune(self, now):
        if self.memory_duration <= 0.0:
            return
        expired = [
            key for key, cell in self.cells.items()
            if now - cell.get('last_seen', now) > self.memory_duration
        ]
        for key in expired:
            del self.cells[key]

    def publish_cloud(self):
        now = time.monotonic()
        self.prune(now)

        points = [
            (cell['x'], cell['y'], cell['z'])
            for cell in self.cells.values()
            if (
                cell.get('hits', 0) >= self.min_hits
                and now - cell.get('first_seen', now) >= self.min_confirm_age
            )
        ]

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        msg.data = b''.join(struct.pack('<fff', *point) for point in points)
        self.cloud_pub.publish(msg)

    def clear_callback(self, request, response):
        del request
        self.cells.clear()
        self.publish_cloud()
        self.get_logger().info('Cleared ultrasonic obstacle memory')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicMemory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
