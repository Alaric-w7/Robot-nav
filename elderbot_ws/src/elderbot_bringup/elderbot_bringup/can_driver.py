#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster
from math import sin, cos, pi
import time
import os
import ctypes
from ctypes import *
from ament_index_python.packages import get_package_prefix

# ==========================================
# 1. 底层 CAN 库配置 (ctypes)
# ==========================================
VCI_USBCAN2 = 4
STATUS_OK = 1
BAUD_T0 = 0x00
BAUD_T1 = 0x14

class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode", c_uint), ("AccMask", c_uint), ("Reserved", c_uint),
                ("Filter", c_ubyte), ("Timing0", c_ubyte), ("Timing1", c_ubyte), ("Mode", c_ubyte)]
class VCI_CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint), ("TimeStamp", c_uint), ("TimeFlag", c_ubyte), ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte), ("ExternFlag", c_ubyte), ("DataLen", c_ubyte),
                ("Data", c_ubyte*8), ("Reserved", c_ubyte*3)]


class VCI_CAN_STATUS(Structure):
    _fields_ = [("ErrInterrupt", c_ubyte), ("regMode", c_ubyte), ("regStatus", c_ubyte),
                ("regALCapture", c_ubyte), ("regECCapture", c_ubyte), ("regEWLimit", c_ubyte),
                ("regRECounter", c_ubyte), ("regTECounter", c_ubyte)]


class VCI_ERR_INFO(Structure):
    _fields_ = [("ErrCode", c_uint), ("Passive_ErrData", c_ubyte * 3), ("ArLost_ErrData", c_ubyte)]

class CanInterface:
    def __init__(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        so_paths = [
            os.path.join(current_dir, 'libcontrolcan.so'),
        ]
        try:
            prefix = get_package_prefix('elderbot_bringup')
            so_paths.insert(0, os.path.join(prefix, 'lib', 'elderbot_bringup', 'libcontrolcan.so'))
        except Exception:
            pass
        so_path = None
        for candidate in so_paths:
            if os.path.exists(candidate):
                so_path = candidate
                break
        try:
            if so_path is None:
                raise OSError("library not found")
            self.canDLL = cdll.LoadLibrary(so_path)
        except OSError:
            print(f"Error: 找不到库文件，已尝试: {', '.join(so_paths)}")
            exit(1)

        if self.canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK:
            print("Error: 打开设备失败 (请确保是在 root 或 chmod 777 下运行)")
            exit(1)

        vci_initconfig = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 0, BAUD_T0, BAUD_T1, 0)
        self.canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_initconfig))
        self.canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)

        # CAN2 通道用 250Kbps (电池 BMS 使用 250K)
        vci_initconfig_can2 = VCI_INIT_CONFIG(0x00000000, 0xFFFFFFFF, 0, 0, 0x01, 0x1C, 0)
        self.canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 1, byref(vci_initconfig_can2))
        self.canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 1)

        nmt_data = (c_ubyte*8)(0x01, 0x00, 0,0,0,0,0,0)
        vci_can_obj = VCI_CAN_OBJ(0x000, 0, 0, 0, 0, 0, 2, nmt_data, (c_ubyte*3)(0,0,0))
        self.canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)

        self.enable_motor(1)
        self.enable_motor(2)
        print(">>> CAN 初始化完成，电机已使能")

    def enable_motor(self, node_id):
        self.send_sdo(node_id, 0x6060, 0x00, [3], 1)
        self.send_sdo(node_id, 0x6040, 0x00, [0x06, 0x00], 2)
        self.send_sdo(node_id, 0x6040, 0x00, [0x07, 0x00], 2)
        self.send_sdo(node_id, 0x6040, 0x00, [0x0F, 0x00], 2)

    def disable_motor(self, node_id):
        self.send_sdo(node_id, 0x6040, 0x00, [0x00, 0x00], 2)
        print(f"电机 {node_id} 已解除使能 (Relaxed)")

    def send_sdo(self, node_id, index, sub, data, length):
        send_id = 0x600 + node_id
        cmd = 0x2F if length == 1 else (0x2B if length == 2 else 0x23)
        payload_list = [cmd, index & 0xFF, (index >> 8) & 0xFF, sub] + data
        while len(payload_list) < 8: payload_list.append(0)
        ubyte_array = c_ubyte*8
        payload = ubyte_array(*payload_list)
        vci_can_obj = VCI_CAN_OBJ(send_id, 0, 0, 0, 0, 0, 8, payload, (c_ubyte*3)(0,0,0))
        self.canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
        time.sleep(0.01)

    def send_speed(self, node_id, rpm):
        send_id = 0x600 + node_id
        val = int(rpm)
        data = [0x23, 0xFF, 0x60, 0x00, val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0xFF, (val >> 24) & 0xFF]
        ubyte_array = c_ubyte*8
        payload = ubyte_array(*data)
        vci_can_obj = VCI_CAN_OBJ(send_id, 0, 0, 0, 0, 0, 8, payload, (c_ubyte*3)(0,0,0))
        self.canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)

    def get_motor_positions(self):
        self._send_query_pos(1)
        self._send_query_pos(2)

        pos_1 = None
        pos_2 = None
        receive_calls = 0
        receive_packets = 0
        sample_frames = []

        for _ in range(20):
            rx_obj = (VCI_CAN_OBJ * 50)()
            num = self.canDLL.VCI_Receive(VCI_USBCAN2, 0, 0, byref(rx_obj), 50, 0)
            receive_calls += 1

            if num > 0:
                receive_packets += int(num)
                for i in range(num):
                    can_id = rx_obj[i].ID
                    data = rx_obj[i].Data

                    if len(sample_frames) < 8:
                        sample_frames.append(
                            f"0x{can_id:03X}[{rx_obj[i].DataLen}] "
                            f"{data[0]:02X} {data[1]:02X} {data[2]:02X} {data[3]:02X}"
                        )

                    if data[1] == 0x63 and data[2] == 0x60:
                        p = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24)
                        if p & 0x80000000: p -= 0x100000000

                        if can_id == 0x581:
                            pos_1 = p
                        elif can_id == 0x582:
                            pos_2 = p

            if pos_1 is not None and pos_2 is not None:
                break

            time.sleep(0.001)

        self.last_motor_query_debug = {
            'receive_calls': receive_calls,
            'receive_packets': receive_packets,
            'sample_frames': sample_frames,
        }
        return pos_1, pos_2

    def get_motor_diagnostics(self):
        pending = self.canDLL.VCI_GetReceiveNum(VCI_USBCAN2, 0, 0)

        status = VCI_CAN_STATUS()
        status_ret = self.canDLL.VCI_ReadCANStatus(VCI_USBCAN2, 0, 0, byref(status))

        err = VCI_ERR_INFO()
        err_ret = self.canDLL.VCI_ReadErrInfo(VCI_USBCAN2, 0, 0, byref(err))

        return {
            'pending': int(pending),
            'last_query': getattr(self, 'last_motor_query_debug', {}),
            'status_ret': int(status_ret),
            'status': {
                'err_interrupt': int(status.ErrInterrupt),
                'reg_mode': int(status.regMode),
                'reg_status': int(status.regStatus),
                'reg_al_capture': int(status.regALCapture),
                'reg_ec_capture': int(status.regECCapture),
                'reg_ew_limit': int(status.regEWLimit),
                'rx_err': int(status.regRECounter),
                'tx_err': int(status.regTECounter),
            },
            'err_ret': int(err_ret),
            'err_code': int(err.ErrCode),
            'passive_err_data': [int(v) for v in err.Passive_ErrData],
            'ar_lost_data': int(err.ArLost_ErrData),
        }

    def _send_query_pos(self, node_id):
        send_id = 0x600 + node_id
        data = [0x40, 0x63, 0x60, 0x00, 0, 0, 0, 0]
        ubyte_array = c_ubyte*8
        payload = ubyte_array(*data)
        vci_can_obj = VCI_CAN_OBJ(send_id, 0, 0, 0, 0, 0, 8, payload, (c_ubyte*3)(0,0,0))
        self.canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)

    # ==========================================
    # 电池 BMS 通讯 (CAN2 通道)
    # ==========================================
    def send_battery_query(self):
        """发送电池查询帧到 CAN2"""
        query_data = (c_ubyte * 8)(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
        query_obj = VCI_CAN_OBJ(
            0x0400FF80,  # 查询帧 ID
            0, 0, 0, 0,
            1,  # ExternFlag = 1 (扩展帧)
            8,
            query_data,
            (c_ubyte * 3)(0, 0, 0)
        )
        self.canDLL.VCI_Transmit(VCI_USBCAN2, 0, 1, byref(query_obj), 1)

    def read_battery_data(self):
        """从 CAN2 读取电池数据，返回 (voltage, current, soc, temperature) 或 None"""
        rx_obj = (VCI_CAN_OBJ * 50)()
        num = self.canDLL.VCI_Receive(VCI_USBCAN2, 0, 1, byref(rx_obj), 50, 0)

        result = None
        if num > 0:
            for i in range(num):
                can_id = rx_obj[i].ID
                data = rx_obj[i].Data
                extern_flag = rx_obj[i].ExternFlag

                if not extern_flag:
                    continue

                # 总信息帧 0x04028001: 电压/电流/SOC
                if can_id == 0x04028001:
                    voltage = ((data[0] << 8) | data[1]) * 0.1
                    current_raw = (data[2] << 8) | data[3]
                    current = (current_raw - 30000) * 0.1
                    soc = ((data[4] << 8) | data[5]) * 0.1
                    result = ('main', voltage, current, soc)

                # 温度帧 0x04018001: 单体温度
                elif can_id == 0x04018001:
                    # Data1: 第一个温度传感器 (偏移40)
                    if data[1] != 0xFF:
                        temp = float(data[1] - 40)
                        result = ('temp', temp)

                # 总信息1 0x04038001: MOS温度
                elif can_id == 0x04038001:
                    # Data4: MOS温度 (偏移40)
                    if data[4] != 0xFF:
                        temp = float(data[4] - 40)
                        result = ('mos_temp', temp)

        return result

# ==========================================
# 2. ROS 2 节点逻辑
# ==========================================

class CanDriverNode(Node):
    def __init__(self):
        super().__init__('can_driver_node')
        self.declare_parameter('publish_odom_tf', False)
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').get_parameter_value().bool_value

        self.wheel_diameter = 0.13
        self.wheel_separation = 0.37
        self.ticks_per_rev = 5733.0

        self.dir_left = 1
        self.dir_right = -1

        self.can = CanInterface()

        self.odom_pub = self.create_publisher(Odometry, 'odom/unfiltered', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_odom_tf else None
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.05, self.timer_callback)

        # 电池监控定时器
        self.create_timer(2.0, self.battery_query_callback)  # 每2秒发送查询帧
        self.create_timer(0.2, self.battery_read_callback)   # 每200ms读取数据
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.battery_soc = 0.0
        self.battery_temperature = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.last_left_ticks = None
        self.last_right_ticks = None

        self.max_rpm_rate = 3000.0
        self.target_rpm_left = 0.0
        self.target_rpm_right = 0.0
        self.current_rpm_left = 0.0
        self.current_rpm_right = 0.0

        self.debug_count = 0

        self.get_logger().info("ElderBot CAN driver started, waiting for commands...")

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        v_left = linear - (angular * self.wheel_separation / 2.0)
        v_right = linear + (angular * self.wheel_separation / 2.0)

        rpm_left = (v_left * 60.0) / (pi * self.wheel_diameter)
        rpm_right = (v_right * 60.0) / (pi * self.wheel_diameter)

        self.target_rpm_left = rpm_left * self.dir_left
        self.target_rpm_right = rpm_right * self.dir_right

    def timer_callback(self):
        current_time_for_ramp = self.get_clock().now()
        if not hasattr(self, 'last_ramp_time'):
            self.last_ramp_time = current_time_for_ramp
        dt_cmd = (current_time_for_ramp - self.last_ramp_time).nanoseconds / 1e9
        self.last_ramp_time = current_time_for_ramp
        if dt_cmd <= 0.0 or dt_cmd > 0.5:
            dt_cmd = 0.05
        max_step = self.max_rpm_rate * dt_cmd
        def _ramp(curr, target):
            delta = target - curr
            if delta > max_step:
                return curr + max_step
            if delta < -max_step:
                return curr - max_step
            return target

        self.current_rpm_left = _ramp(self.current_rpm_left, self.target_rpm_left)
        self.current_rpm_right = _ramp(self.current_rpm_right, self.target_rpm_right)

        self.can.send_speed(1, self.current_rpm_left)
        self.can.send_speed(2, self.current_rpm_right)

        curr_left, curr_right = self.can.get_motor_positions()

        if self.last_left_ticks is None or self.last_right_ticks is None:
            if curr_left is not None: self.last_left_ticks = curr_left
            if curr_right is not None: self.last_right_ticks = curr_right
            return

        if curr_left is None or curr_right is None:
            if not hasattr(self, 'warn_count'): self.warn_count = 0
            self.warn_count += 1
            if self.warn_count % 40 == 0:
                diag = self.can.get_motor_diagnostics()
                self.get_logger().warn(
                    "Failed to read encoders! "
                    f"Left: {curr_left}, Right: {curr_right}, "
                    f"pending={diag['pending']}, err_code=0x{diag['err_code']:08X}, "
                    f"rx_err={diag['status']['rx_err']}, tx_err={diag['status']['tx_err']}, "
                    f"reg_status=0x{diag['status']['reg_status']:02X}, "
                    f"err_interrupt=0x{diag['status']['err_interrupt']:02X}, "
                    f"receive_calls={diag['last_query'].get('receive_calls', 0)}, "
                    f"receive_packets={diag['last_query'].get('receive_packets', 0)}, "
                    f"sample_frames={diag['last_query'].get('sample_frames', [])}"
                )
            return

        self.warn_count = 0

        delta_L_ticks = (curr_left - self.last_left_ticks) * self.dir_left
        delta_R_ticks = (curr_right - self.last_right_ticks) * self.dir_right

        self.last_left_ticks = curr_left
        self.last_right_ticks = curr_right

        d_left = -1.0 *(delta_L_ticks / self.ticks_per_rev) * (pi * self.wheel_diameter)
        d_right = -1.0 *(delta_R_ticks / self.ticks_per_rev) * (pi * self.wheel_diameter)


        d_center = (d_left + d_right) / 2.0
        d_th = (d_right - d_left) / self.wheel_separation

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt > 0:
            self.x += d_center * cos(self.th)
            self.y += d_center * sin(self.th)
            self.th += d_th
            yaw = self.th

            self.debug_count += 1
            if self.debug_count % 10 == 0:
                self.get_logger().info(
                    f"[Data] Enc: L={curr_left} R={curr_right} | Pose: x={self.x:.2f}, y={self.y:.2f}, th={self.th:.2f}"
                )

            if self.tf_broadcaster is not None:
                t = TransformStamped()
                t.header.stamp = current_time.to_msg()
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_footprint'
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation.z = sin(yaw / 2.0)
                t.transform.rotation.w = cos(yaw / 2.0)
                self.tf_broadcaster.sendTransform(t)

            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = sin(yaw / 2.0)
            odom.pose.pose.orientation.w = cos(yaw / 2.0)

            odom.twist.twist.linear.x = d_center / dt
            odom.twist.twist.angular.z = d_th / dt
            self.odom_pub.publish(odom)

    def battery_query_callback(self):
        """定时发送电池查询帧"""
        self.can.send_battery_query()

    def battery_read_callback(self):
        """定时读取电池数据并发布"""
        # 多次读取以获取不同类型的帧
        for _ in range(5):
            result = self.can.read_battery_data()
            if result:
                if result[0] == 'main':
                    _, self.battery_voltage, self.battery_current, self.battery_soc = result
                elif result[0] == 'temp':
                    _, self.battery_temperature = result
                elif result[0] == 'mos_temp':
                    _, self.battery_temperature = result

        # 发布电池状态
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'battery'
        msg.voltage = self.battery_voltage
        msg.current = self.battery_current
        msg.percentage = self.battery_soc / 100.0
        msg.temperature = self.battery_temperature

        if self.battery_current > 0.1:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif self.battery_current < -0.1:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.battery_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CanDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.can.send_speed(1, 0)
        node.can.send_speed(2, 0)
        time.sleep(0.1)

        node.can.disable_motor(1)
        node.can.disable_motor(2)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
