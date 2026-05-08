#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
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
        self.declare_parameter("trigger_period", 0.10)
        self.declare_parameter("reset_input_before_trigger", False)
        self.declare_parameter("flush_trigger_write", False)
        self.declare_parameter("frame_timeout", 0.5)
        self.declare_parameter("stale_timeout", 1.0)
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
        self.declare_parameter("enabled_channels", [1, 2, 3, 4])
        self.declare_parameter("angles", [0.0, 1.5708, 3.14159, -1.5708])
        self.declare_parameter("scan_range_offsets", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("filter_consecutive_hits", 1)
        self.declare_parameter("filter_release_frames", 1)
        self.declare_parameter("filter_tolerance", 0.25)
        self.declare_parameter(
            "ground_reject_windows",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

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
        self.trigger_period = float(self.get_parameter("trigger_period").value)
        self.reset_input_before_trigger = bool(
            self.get_parameter("reset_input_before_trigger").value
        )
        self.flush_trigger_write = bool(
            self.get_parameter("flush_trigger_write").value
        )
        self.frame_timeout = float(self.get_parameter("frame_timeout").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout").value)
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
        self.enabled_channels = self._parse_enabled_channels(
            self.get_parameter("enabled_channels").value
        )
        self.angles = [float(angle) for angle in self.get_parameter("angles").value]
        self.scan_range_offsets = self._normalize_float_list(
            self.get_parameter("scan_range_offsets").value,
            length=4,
            default=0.0,
        )
        self.filter_consecutive_hits = max(
            1, int(self.get_parameter("filter_consecutive_hits").value)
        )
        self.filter_release_frames = max(
            1, int(self.get_parameter("filter_release_frames").value)
        )
        self.filter_tolerance = max(
            0.0, float(self.get_parameter("filter_tolerance").value)
        )
        self.ground_reject_windows = self._normalize_reject_windows(
            self.get_parameter("ground_reject_windows").value
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

        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=self.bytesize,
            parity=self.parity,
            stopbits=self.stopbits,
            timeout=self.timeout,
        )

        self._rx_buffer = bytearray()
        self._last_dists_m = [None, None, None, None]
        self._candidate_dists_m = [None, None, None, None]
        self._candidate_counts = [0, 0, 0, 0]
        self._published_dists_m = [None, None, None, None]
        self._missing_counts = [0, 0, 0, 0]
        self._last_frame_time = None
        self._last_trigger_time = 0.0
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
            f"SendTrigger: {self.send_trigger}, TriggerHex: {self.trigger_bytes.hex(' ').upper()}, "
            f"TriggerPeriod: {self.trigger_period}, ResetInputBeforeTrigger: "
            f"{self.reset_input_before_trigger}, FlushTriggerWrite: "
            f"{self.flush_trigger_write}, FrameTimeout: {self.frame_timeout}, "
            f"StaleTimeout: {self.stale_timeout}"
        )
        self.get_logger().info(
            "Enabled channels: "
            f"{[index + 1 for index in self.enabled_channels]}, "
            f"angles(rad): {self.angles}, scan offsets(m): {self.scan_range_offsets}"
        )
        self.get_logger().info(
            "Filter: "
            f"consecutive_hits={self.filter_consecutive_hits}, "
            f"release_frames={self.filter_release_frames}, "
            f"tolerance={self.filter_tolerance}, "
            f"ground_reject_windows={self.ground_reject_windows}"
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
    def _parse_enabled_channels(cls, value):
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

    @classmethod
    def _normalize_reject_windows(cls, value):
        values = cls._normalize_float_list(value, length=8, default=0.0)
        windows = []
        for index in range(0, 8, 2):
            reject_min = values[index]
            reject_max = values[index + 1]
            if reject_max > reject_min:
                windows.append((reject_min, reject_max))
            else:
                windows.append((0.0, 0.0))
        return windows

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
        ranges = [float("nan")] * num_points
        half_width_deg = max(0, int(round(math.degrees(self.fov) / 2.0)))

        for index, (dist, angle) in enumerate(zip(dists_m, self.angles)):
            if index not in self.enabled_channels:
                continue

            scan_dist = (
                float("inf") if dist is None else dist + self.scan_range_offsets[index]
            )
            deg = int(round(math.degrees(angle))) % 360
            for offset in range(-half_width_deg, half_width_deg + 1):
                scan_index = (deg + offset) % 360
                current = ranges[scan_index]
                if not math.isfinite(current):
                    ranges[scan_index] = scan_dist
                elif math.isfinite(scan_dist):
                    ranges[scan_index] = min(current, scan_dist)

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

    def send_trigger_if_needed(self):
        if not self.send_trigger:
            return

        now = time.monotonic()
        if self.trigger_period > 0.0 and now - self._last_trigger_time < self.trigger_period:
            return
        self._last_trigger_time = now

        if self.reset_input_before_trigger:
            self._rx_buffer.clear()
            self.serial_port.reset_input_buffer()

        self.serial_port.write(self.trigger_bytes)
        if self.flush_trigger_write:
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

    def poll_latest_frame(self):
        self.send_trigger_if_needed()

        waiting = self.serial_port.in_waiting
        if waiting:
            chunk = self.serial_port.read(waiting)
            if chunk:
                self._rx_buffer.extend(chunk)

        latest_frame = None
        while True:
            frame = self._try_extract_frame()
            if frame is None:
                break
            latest_frame = frame

        return latest_frame

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

    def poll_distances_mm(self):
        frame = self.poll_latest_frame()
        if frame is None:
            return None
        if self.log_raw_frames:
            self.throttle_info(
                f"raw frame: {frame.hex(' ').upper()}",
                interval_sec=0.2,
            )
        return self.parse_frame(frame)

    def raw_mm_to_dists_m(self, raw_mm):
        dists_m = []
        for index, value_mm in enumerate(raw_mm):
            if index not in self.enabled_channels:
                dists_m.append(None)
                continue
            if self.valid_mm(value_mm):
                dist_m = self.mm_to_m(value_mm)
                if self.min_range <= dist_m <= self.max_range:
                    reject_min, reject_max = self.ground_reject_windows[index]
                    if reject_min <= dist_m <= reject_max and reject_max > reject_min:
                        dists_m.append(None)
                    else:
                        dists_m.append(dist_m)
                else:
                    dists_m.append(None)
            else:
                dists_m.append(None)
        return dists_m

    def filter_distances(self, raw_dists_m):
        filtered = []
        for index, dist in enumerate(raw_dists_m):
            if index not in self.enabled_channels:
                self._published_dists_m[index] = None
                filtered.append(None)
                continue

            if dist is None:
                self._missing_counts[index] += 1
                self._candidate_dists_m[index] = None
                self._candidate_counts[index] = 0
                if self._missing_counts[index] >= self.filter_release_frames:
                    self._published_dists_m[index] = None
                filtered.append(self._published_dists_m[index])
                continue

            self._missing_counts[index] = 0
            candidate = self._candidate_dists_m[index]
            if (
                candidate is None
                or (
                    self.filter_tolerance > 0.0
                    and abs(dist - candidate) > self.filter_tolerance
                )
            ):
                self._candidate_dists_m[index] = dist
                self._candidate_counts[index] = 1
            else:
                count = self._candidate_counts[index]
                self._candidate_dists_m[index] = (
                    candidate * count + dist
                ) / (count + 1)
                self._candidate_counts[index] = count + 1

            if self._candidate_counts[index] >= self.filter_consecutive_hits:
                self._published_dists_m[index] = self._candidate_dists_m[index]

            filtered.append(self._published_dists_m[index])

        return filtered

    def read_and_publish(self):
        try:
            raw_mm = self.poll_distances_mm()
            stamp = self.get_clock().now().to_msg()

            now = time.monotonic()
            if raw_mm is not None:
                raw_dists_m = self.raw_mm_to_dists_m(raw_mm)
                self._last_dists_m = self.filter_distances(raw_dists_m)
                self._last_frame_time = now

            dists_m = self._last_dists_m
            stale = (
                self._last_frame_time is None
                or (
                    self.stale_timeout > 0.0
                    and now - self._last_frame_time > self.stale_timeout
                )
            )
            if stale:
                dists_m = [None, None, None, None]

            self.publish_ranges(dists_m, stamp)

            if self.publish_scan and self.scan_pub is not None:
                self.publish_laserscan(dists_m, stamp)

            readable = ["None" if dist is None else round(dist, 3) for dist in dists_m]
            self.throttle_info(
                f"ultrasonic(m): {readable}, fresh_frame={raw_mm is not None}, stale={stale}"
            )

        except Exception as exc:
            self.throttle_warn(f"Read failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DypE08PyserialNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            serial_port = getattr(node, "serial_port", None)
            if serial_port is not None and getattr(serial_port, "is_open", False):
                serial_port.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
