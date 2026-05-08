#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import time

import minimalmodbus
import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class DypE08Rs485Node(Node):
    def __init__(self):
        super().__init__("dyp_e08_rs485_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("slave_addr", 1)
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("bytesize", 8)
        self.declare_parameter("parity", "N")
        self.declare_parameter("stopbits", 1)
        self.declare_parameter("timeout", 0.2)
        self.declare_parameter("close_port_after_each_call", False)
        self.declare_parameter("modbus_debug", False)
        self.declare_parameter("register_address", 0x0106)
        self.declare_parameter("register_count", 4)
        self.declare_parameter("function_code", 3)
        self.declare_parameter("min_range", 0.02)
        self.declare_parameter("max_range", 4.50)
        self.declare_parameter("field_of_view", 0.26)
        self.declare_parameter("rate", 10.0)
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
        self.declare_parameter("enabled_channels", [1, 2, 3, 4])
        self.declare_parameter("angles", [0.0, 1.5708, 3.14159, -1.5708])
        self.declare_parameter("scan_range_offsets", [0.0, 0.0, 0.0, 0.0])

        self.port = self.get_parameter("port").value
        self.slave_addr = int(self.get_parameter("slave_addr").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.bytesize = int(self.get_parameter("bytesize").value)
        self.parity = self._parse_parity(self.get_parameter("parity").value)
        self.stopbits = int(self.get_parameter("stopbits").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.close_port_after_each_call = bool(
            self.get_parameter("close_port_after_each_call").value
        )
        self.modbus_debug = bool(self.get_parameter("modbus_debug").value)
        self.register_address = int(self.get_parameter("register_address").value)
        self.register_count = int(self.get_parameter("register_count").value)
        self.function_code = int(self.get_parameter("function_code").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.fov = float(self.get_parameter("field_of_view").value)
        self.rate_hz = float(self.get_parameter("rate").value)
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
        self.enabled_channels = self._parse_enabled_channels(
            self.get_parameter("enabled_channels").value
        )
        self.angles = [float(angle) for angle in self.get_parameter("angles").value]
        self.scan_range_offsets = self._normalize_float_list(
            self.get_parameter("scan_range_offsets").value,
            length=4,
            default=0.0,
        )

        self.pub1 = self.create_publisher(Range, self.topic_1, 10)
        self.pub2 = self.create_publisher(Range, self.topic_2, 10)
        self.pub3 = self.create_publisher(Range, self.topic_3, 10)
        self.pub4 = self.create_publisher(Range, self.topic_4, 10)
        self.scan_pub = None
        if self.publish_scan:
            self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        if not os.path.exists(self.port):
            self.get_logger().warn(f"Serial port does not exist yet: {self.port}")

        self.instrument = minimalmodbus.Instrument(
            self.port,
            self.slave_addr,
            mode=minimalmodbus.MODE_RTU,
            close_port_after_each_call=self.close_port_after_each_call,
            debug=self.modbus_debug,
        )
        self.instrument.serial.baudrate = self.baudrate
        self.instrument.serial.bytesize = self.bytesize
        self.instrument.serial.parity = self.parity
        self.instrument.serial.stopbits = self.stopbits
        self.instrument.serial.timeout = self.timeout
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True

        self._last_info_log_time = 0.0
        self._last_warn_log_time = 0.0

        timer_period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(timer_period, self.read_and_publish)

        self.get_logger().info("DYP-E08 RS485 ROS2 node started")
        self.get_logger().info(
            "Port: "
            f"{self.port}, Slave: {self.slave_addr}, Baud: {self.baudrate}, "
            f"Bytesize: {self.bytesize}, Parity: {self.parity}, Stopbits: {self.stopbits}, "
            f"Timeout: {self.timeout}, CloseEachCall: {self.close_port_after_each_call}"
        )
        self.get_logger().info(
            f"Modbus read config: func={self.function_code}, "
            f"start=0x{self.register_address:04X}, count={self.register_count}"
        )
        self.get_logger().info(
            "Enabled channels: "
            f"{[index + 1 for index in self.enabled_channels]}, "
            f"angles(rad): {self.angles}, scan offsets(m): {self.scan_range_offsets}"
        )

    def read_distances_mm(self):
        """
        连续读取 4 个寄存器:
        0x0106, 0x0107, 0x0108, 0x0109
        返回 [d1, d2, d3, d4]，单位 mm
        """
        values = self.instrument.read_registers(
            registeraddress=self.register_address,
            number_of_registers=self.register_count,
            functioncode=self.function_code,
        )
        if len(values) != 4:
            raise ValueError(
                f"Expected 4 registers, got {len(values)}. "
                "Check register_count and sensor protocol."
            )
        return values

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
    def _parse_enabled_channels(value):
        if isinstance(value, str):
            raw_channels = value.replace(" ", "").split(",")
        else:
            raw_channels = value

        channels = []
        for channel in raw_channels:
            try:
                index = int(channel) - 1
            except (TypeError, ValueError):
                continue
            if 0 <= index < 4 and index not in channels:
                channels.append(index)
        return channels

    @staticmethod
    def _normalize_float_list(value, length, default):
        if isinstance(value, str):
            raw_values = value.replace(" ", "").split(",")
        else:
            raw_values = value

        values = []
        for item in raw_values:
            try:
                values.append(float(item))
            except (TypeError, ValueError):
                values.append(default)

        while len(values) < length:
            values.append(default)
        return values[:length]

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
        half_width_deg = max(0, int(round(math.degrees(self.fov) / 2.0)))

        for index, (dist, angle) in enumerate(zip(dists_m, self.angles)):
            if index not in self.enabled_channels:
                continue
            if dist is None:
                continue

            scan_dist = dist + self.scan_range_offsets[index]
            deg = int(round(math.degrees(angle))) % 360
            for offset in range(-half_width_deg, half_width_deg + 1):
                scan_index = (deg + offset) % 360
                ranges[scan_index] = min(ranges[scan_index], scan_dist)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = 2.0 * math.pi / num_points
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.0
        scan.range_min = self.min_range
        scan.range_max = self.max_range + max([0.0] + self.scan_range_offsets)
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

    def read_and_publish(self):
        try:
            raw_mm = self.read_distances_mm()
            stamp = self.get_clock().now().to_msg()

            dists_m = []
            for index, value_mm in enumerate(raw_mm):
                if index not in self.enabled_channels:
                    dists_m.append(None)
                    continue
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
        node = DypE08Rs485Node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            serial_port = getattr(getattr(node, "instrument", None), "serial", None)
            if serial_port is not None and getattr(serial_port, "is_open", False):
                serial_port.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
