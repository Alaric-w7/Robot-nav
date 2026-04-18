#!/usr/bin/env python3

import math
import os
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class DypE08PyserialNode(Node):
    FRAME_HEADER = 0xFF
    FRAME_SIZE = 10

    def __init__(self):
        super().__init__("dyp_e08_pyserial_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("bytesize", 8)
        self.declare_parameter("parity", "N")
        self.declare_parameter("stopbits", 1)
        self.declare_parameter("timeout", 0.2)
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("send_trigger", True)
        self.declare_parameter("trigger_hex", "FF")
        self.declare_parameter("frame_timeout", 0.5)
        self.declare_parameter("log_raw_frames", False)
        self.declare_parameter("min_range", 0.02)
        self.declare_parameter("max_range", 4.50)
        self.declare_parameter("field_of_view", 0.26)
        self.declare_parameter("frame_id_1", "ultrasonic_1")
        self.declare_parameter("frame_id_2", "ultrasonic_2")
        self.declare_parameter("frame_id_3", "ultrasonic_3")
        self.declare_parameter("frame_id_4", "ultrasonic_4")
        self.declare_parameter("scan_frame", "base_link")
        self.declare_parameter("topic_1", "/ultrasonic_1")
        self.declare_parameter("topic_2", "/ultrasonic_2")
        self.declare_parameter("topic_3", "/ultrasonic_3")
        self.declare_parameter("topic_4", "/ultrasonic_4")
        self.declare_parameter("scan_topic", "/scan_ultrasonic")
        self.declare_parameter("publish_scan", True)
        self.declare_parameter("angles", [0.0, 1.5708, 3.14159, -1.5708])

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.bytesize = int(self.get_parameter("bytesize").value)
        self.parity = self._parse_parity(self.get_parameter("parity").value)
        self.stopbits = int(self.get_parameter("stopbits").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.rate_hz = float(self.get_parameter("rate").value)
        self.send_trigger = bool(self.get_parameter("send_trigger").value)
        self.trigger_bytes = self._parse_hex_bytes(
            self.get_parameter("trigger_hex").value
        )
        self.frame_timeout = float(self.get_parameter("frame_timeout").value)
        self.log_raw_frames = bool(self.get_parameter("log_raw_frames").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.fov = float(self.get_parameter("field_of_view").value)
        self.frame_id_1 = self.get_parameter("frame_id_1").value
        self.frame_id_2 = self.get_parameter("frame_id_2").value
        self.frame_id_3 = self.get_parameter("frame_id_3").value
        self.frame_id_4 = self.get_parameter("frame_id_4").value
        self.scan_frame = self.get_parameter("scan_frame").value
        self.topic_1 = self.get_parameter("topic_1").value
        self.topic_2 = self.get_parameter("topic_2").value
        self.topic_3 = self.get_parameter("topic_3").value
        self.topic_4 = self.get_parameter("topic_4").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.publish_scan = bool(self.get_parameter("publish_scan").value)
        self.angles = [float(angle) for angle in self.get_parameter("angles").value]

        self.pub1 = self.create_publisher(Range, self.topic_1, 10)
        self.pub2 = self.create_publisher(Range, self.topic_2, 10)
        self.pub3 = self.create_publisher(Range, self.topic_3, 10)
        self.pub4 = self.create_publisher(Range, self.topic_4, 10)
        self.scan_pub = None
        if self.publish_scan:
            self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        if not os.path.exists(self.port):
            self.get_logger().warn(f"Serial port does not exist yet: {self.port}")

        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=self.bytesize,
            parity=self.parity,
            stopbits=self.stopbits,
            timeout=self.timeout,
        )

        self._rx_buffer = bytearray()
        self._last_info_log_time = 0.0
        self._last_warn_log_time = 0.0

        timer_period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(timer_period, self.read_and_publish)

        self.get_logger().info("DYP-E08 pyserial ROS2 node started")
        self.get_logger().info(
            "Port: "
            f"{self.port}, Baud: {self.baudrate}, Bytesize: {self.bytesize}, "
            f"Parity: {self.parity}, Stopbits: {self.stopbits}, Timeout: {self.timeout}"
        )
        self.get_logger().info(
            f"SendTrigger: {self.send_trigger}, "
            f"TriggerHex: {self.trigger_bytes.hex(' ').upper()}, "
            f"FrameTimeout: {self.frame_timeout}"
        )

    @staticmethod
    def _parse_parity(value):
        parity_map = {
            "N": serial.PARITY_NONE,
            "E": serial.PARITY_EVEN,
            "O": serial.PARITY_ODD,
        }
        normalized = str(value).strip().upper()
        if normalized not in parity_map:
            raise ValueError(
                f"Unsupported parity '{value}'. Use one of: N, E, O."
            )
        return parity_map[normalized]

    @staticmethod
    def _parse_hex_bytes(value):
        normalized = str(value).replace("0x", "").replace("0X", "")
        normalized = normalized.replace(" ", "").replace(",", "")
        if len(normalized) == 0 or len(normalized) % 2 != 0:
            raise ValueError(
                f"Invalid trigger_hex '{value}'. Expected even-length hex bytes."
            )
        return bytes.fromhex(normalized)

    @classmethod
    def _frame_checksum_ok(cls, frame):
        if len(frame) != cls.FRAME_SIZE:
            return False
        if frame[0] != cls.FRAME_HEADER:
            return False
        expected = sum(frame[:-1]) & 0xFF
        return expected == frame[-1]

    @staticmethod
    def valid_mm(value):
        if value in (0xFFFF, 0xEEEE):
            return False
        if value <= 0:
            return False
        return True

    @staticmethod
    def mm_to_m(value_mm):
        return value_mm / 1000.0

    def make_range_msg(self, dist_m, frame_id, stamp):
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = float("inf") if dist_m is None else dist_m
        return msg

    def publish_ranges(self, dists_m, stamp):
        msgs = [
            self.make_range_msg(dists_m[0], self.frame_id_1, stamp),
            self.make_range_msg(dists_m[1], self.frame_id_2, stamp),
            self.make_range_msg(dists_m[2], self.frame_id_3, stamp),
            self.make_range_msg(dists_m[3], self.frame_id_4, stamp),
        ]
        self.pub1.publish(msgs[0])
        self.pub2.publish(msgs[1])
        self.pub3.publish(msgs[2])
        self.pub4.publish(msgs[3])

    def publish_laserscan(self, dists_m, stamp):
        num_points = 360
        ranges = [float("inf")] * num_points

        for dist, angle in zip(dists_m, self.angles):
            if dist is None:
                continue

            deg = int(round(math.degrees(angle))) % 360
            ranges[deg] = dist

            for offset in [-2, -1, 1, 2]:
                ranges[(deg + offset) % 360] = dist

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = 2.0 * math.pi / num_points
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.0
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = ranges

        self.scan_pub.publish(scan)

    def throttle_info(self, message, interval_sec=1.0):
        now = time.monotonic()
        if now - self._last_info_log_time >= interval_sec:
            self.get_logger().info(message)
            self._last_info_log_time = now

    def throttle_warn(self, message, interval_sec=1.0):
        now = time.monotonic()
        if now - self._last_warn_log_time >= interval_sec:
            self.get_logger().warn(message)
            self._last_warn_log_time = now

    def send_trigger_if_needed(self):
        if not self.send_trigger:
            return
        self.serial_port.reset_input_buffer()
        self.serial_port.write(self.trigger_bytes)
        self.serial_port.flush()

    def _try_extract_frame(self):
        while True:
            header_index = self._rx_buffer.find(bytes([self.FRAME_HEADER]))
            if header_index < 0:
                self._rx_buffer.clear()
                return None

            if header_index > 0:
                del self._rx_buffer[:header_index]

            if len(self._rx_buffer) < self.FRAME_SIZE:
                return None

            candidate = bytes(self._rx_buffer[:self.FRAME_SIZE])
            if self._frame_checksum_ok(candidate):
                del self._rx_buffer[:self.FRAME_SIZE]
                return candidate

            del self._rx_buffer[0]

    def read_frame(self):
        self.send_trigger_if_needed()

        deadline = time.monotonic() + self.frame_timeout
        while time.monotonic() < deadline:
            frame = self._try_extract_frame()
            if frame is not None:
                return frame

            chunk_size = self.serial_port.in_waiting or self.FRAME_SIZE
            chunk = self.serial_port.read(chunk_size)
            if chunk:
                self._rx_buffer.extend(chunk)

        frame = self._try_extract_frame()
        if frame is not None:
            return frame
        raise TimeoutError("Timed out waiting for UART frame")

    @staticmethod
    def parse_frame(frame):
        values = []
        for index in range(1, 9, 2):
            values.append((frame[index] << 8) | frame[index + 1])
        return values

    def read_distances_mm(self):
        frame = self.read_frame()
        if self.log_raw_frames:
            self.throttle_info(
                f"raw frame: {frame.hex(' ').upper()}",
                interval_sec=0.2,
            )
        return self.parse_frame(frame)

    def read_and_publish(self):
        try:
            raw_mm = self.read_distances_mm()
            stamp = self.get_clock().now().to_msg()

            dists_m = []
            for value_mm in raw_mm:
                if self.valid_mm(value_mm):
                    dist_m = self.mm_to_m(value_mm)
                    if self.min_range <= dist_m <= self.max_range:
                        dists_m.append(dist_m)
                    else:
                        dists_m.append(None)
                else:
                    dists_m.append(None)

            self.publish_ranges(dists_m, stamp)

            if self.publish_scan and self.scan_pub is not None:
                self.publish_laserscan(dists_m, stamp)

            readable = ["None" if dist is None else round(dist, 3) for dist in dists_m]
            self.throttle_info(f"ultrasonic(m): {readable}")

        except Exception as exc:
            self.throttle_warn(f"Read failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DypE08PyserialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            serial_port = getattr(node, "serial_port", None)
            if serial_port is not None and getattr(serial_port, "is_open", False):
                serial_port.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
