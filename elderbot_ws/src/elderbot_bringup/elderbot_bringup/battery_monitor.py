#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电池监控节点 - 通过 CAN2 通道读取 BMS 数据
协议: 达锂电子 CAN 协议 V1
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import os
import ctypes
from ctypes import *
from ament_index_python.packages import get_package_prefix

# ==========================================
# CAN 库配置 (与 can_driver.py 相同)
# ==========================================
VCI_USBCAN2 = 4
STATUS_OK = 1
# 250K 波特率 (BMS 使用 250Kbps)
BAUD_T0 = 0x01
BAUD_T1 = 0x1C

class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode", c_uint), ("AccMask", c_uint), ("Reserved", c_uint),
                ("Filter", c_ubyte), ("Timing0", c_ubyte), ("Timing1", c_ubyte), ("Mode", c_ubyte)]

class VCI_CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint), ("TimeStamp", c_uint), ("TimeFlag", c_ubyte), ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte), ("ExternFlag", c_ubyte), ("DataLen", c_ubyte),
                ("Data", c_ubyte*8), ("Reserved", c_ubyte*3)]


class BatteryMonitor(Node):
    """电池监控 ROS2 节点"""

    # BMS CAN ID 定义 (扩展帧, 29位)
    # 实际格式: 0x04XX8001, 其中 XX 是帧类型, 01 是设备地址
    CAN_ID_QUERY = 0x0400FF80       # 查询帧 - 发送此帧后 BMS 才会回复数据
    CAN_ID_TOTAL_INFO = 0x04028001  # 总信息0: 电压/电流/SOC
    CAN_ID_POWER_INFO = 0x04038001  # 总信息1: 功率/温度
    CAN_ID_STATUS_INFO = 0x04068001  # 状态信息0: MOS状态
    CAN_ID_CELL_STAT = 0x04088001   # 状态信息2: 电池节数/容量

    def __init__(self):
        super().__init__('battery_monitor')

        # 参数
        self.declare_parameter('can_channel', 1)  # CAN2 = channel 1
        self.declare_parameter('device_address', 0x01)  # BMS 设备地址
        self.declare_parameter('publish_rate', 1.0)  # 发布频率 Hz

        self.can_channel = self.get_parameter('can_channel').value
        self.device_addr = self.get_parameter('device_address').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # 电池数据
        self.voltage = 0.0  # 总电压 V
        self.current = 0.0  # 电流 A
        self.soc = 0.0  # 电量百分比 0-100
        self.power = 0.0  # 功率 W
        self.temperature = 0.0  # 温度
        self.charging = False  # 充电状态
        self.discharging = False  # 放电状态

        # 加载 CAN 库
        self.canDLL = self._load_can_library()

        # 初始化 CAN (只初始化 CAN2 通道, 不影响 CAN1 的电机通信)
        # 注意: 设备已由 can_driver 打开, 这里只需要确保通道1已初始化
        self._init_can_channel()

        # ROS2 发布者
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)

        # 定时器: 发送查询帧 (每2秒发送一次，BMS 收到后会回复数据)
        self.create_timer(2.0, self.send_query_frame)

        # 定时器: 读取 CAN 数据
        self.create_timer(0.1, self.read_can_data)  # 100ms 读取一次

        # 定时器: 发布电池状态
        self.create_timer(1.0 / self.publish_rate, self.publish_battery_state)

        # 启动时立即发送一次查询帧
        self.send_query_frame()

        self.get_logger().info(f"电池监控节点启动, CAN 通道: {self.can_channel}, 设备地址: 0x{self.device_addr:02X}")

    def _load_can_library(self):
        """加载 CAN 库"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        so_paths = [os.path.join(current_dir, 'libcontrolcan.so')]

        try:
            prefix = get_package_prefix('elderbot_bringup')
            so_paths.insert(0, os.path.join(prefix, 'lib', 'elderbot_bringup', 'libcontrolcan.so'))
        except Exception:
            pass

        for candidate in so_paths:
            if os.path.exists(candidate):
                try:
                    return cdll.LoadLibrary(candidate)
                except OSError:
                    continue

        self.get_logger().error(f"找不到 CAN 库文件: {so_paths}")
        raise RuntimeError("CAN library not found")

    def _init_can_channel(self):
        """初始化 CAN 设备和通道"""
        # 尝试打开设备 (如果 can_driver 没有运行)
        ret = self.canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
        if ret != STATUS_OK:
            self.get_logger().warn("CAN 设备可能已被其他程序打开，尝试共享...")

        vci_initconfig = VCI_INIT_CONFIG(
            0x00000000,  # AccCode - 接收所有
            0xFFFFFFFF,  # AccMask - 不过滤
            0, 0,
            BAUD_T0, BAUD_T1,  # 500Kbps
            0  # 正常模式
        )

        # 初始化 CAN 通道
        ret = self.canDLL.VCI_InitCAN(VCI_USBCAN2, 0, self.can_channel, byref(vci_initconfig))
        if ret != STATUS_OK:
            self.get_logger().warn(f"CAN 通道 {self.can_channel} 初始化返回: {ret}")

        ret = self.canDLL.VCI_StartCAN(VCI_USBCAN2, 0, self.can_channel)
        if ret != STATUS_OK:
            self.get_logger().warn(f"CAN 通道 {self.can_channel} 启动返回: {ret}")

        self.get_logger().info(f"CAN 通道 {self.can_channel} (CAN2) 初始化完成")

    def send_query_frame(self):
        """发送查询帧，触发 BMS 回复数据"""
        query_data = (c_ubyte * 8)(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
        query_obj = VCI_CAN_OBJ(
            self.CAN_ID_QUERY,  # 0x0400FF80
            0, 0, 0, 0,
            1,  # ExternFlag = 1 (扩展帧)
            8,  # DataLen
            query_data,
            (c_ubyte * 3)(0, 0, 0)
        )
        ret = self.canDLL.VCI_Transmit(VCI_USBCAN2, 0, self.can_channel, byref(query_obj), 1)
        if ret != 1:
            self.get_logger().warn("查询帧发送失败")

    def read_can_data(self):
        """读取 CAN 数据"""
        rx_obj = (VCI_CAN_OBJ * 50)()
        num = self.canDLL.VCI_Receive(VCI_USBCAN2, 0, self.can_channel, byref(rx_obj), 50, 0)

        if num > 0:
            for i in range(num):
                can_id = rx_obj[i].ID
                data = rx_obj[i].Data
                extern_flag = rx_obj[i].ExternFlag

                # 只处理扩展帧
                if extern_flag:
                    self._parse_bms_frame(can_id, data)

    def _parse_bms_frame(self, can_id, data):
        """解析 BMS 数据帧"""

        # 总信息0: 电压/电流/SOC (0x04028001)
        if can_id == self.CAN_ID_TOTAL_INFO:
            # Data0-1: 总电压 (0.1V)
            self.voltage = ((data[0] << 8) | data[1]) * 0.1
            # Data2-3: 电流 (0.1A, 有符号, 30000偏移)
            current_raw = (data[2] << 8) | data[3]
            self.current = (current_raw - 30000) * 0.1
            # Data4-5: SOC (0.1%)
            self.soc = ((data[4] << 8) | data[5]) * 0.1

            self.get_logger().debug(f"解析: V={self.voltage:.1f}V, I={self.current:.2f}A, SOC={self.soc:.1f}%")

        # 总信息1: 功率/温度 (0x04038001)
        elif can_id == self.CAN_ID_POWER_INFO:
            # Data0-1: 功率 (W)
            self.power = (data[0] << 8) | data[1]
            # Data4: MOS温度 (偏移40)
            if data[4] != 0xFF:
                self.temperature = data[4] - 40

        # 状态信息0: MOS状态 (0x04068001)
        elif can_id == self.CAN_ID_STATUS_INFO:
            # Data0: 充电MOS状态
            self.charging = (data[0] == 0x01)
            # Data1: 放电MOS状态
            self.discharging = (data[1] == 0x01)

    def publish_battery_state(self):
        """发布电池状态消息"""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'battery'

        msg.voltage = self.voltage
        msg.current = self.current
        msg.percentage = self.soc / 100.0  # 转换为 0-1 范围

        # 根据电流判断充放电状态
        if self.current > 0.1:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif self.current < -0.1:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        self.battery_pub.publish(msg)

        # 定期打印状态
        self.get_logger().info(
            f"[Battery] V={self.voltage:.1f}V, I={self.current:.2f}A, SOC={self.soc:.1f}%"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
