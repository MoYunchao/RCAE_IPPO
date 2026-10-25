#!/usr/bin/env python3
"""
Real Backend - 真实资源监控后端

功能：
- 读取真实电池状态
- 测量真实网络带宽
- 读取真实 CPU 使用率
"""

import time
from typing import Tuple

from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class RealBackend:
    """
    真实后端（推理时使用）
    
    功能：
    - 读取真实电池状态（从硬件接口）
    - 测量真实网络带宽（ping/iperf）
    - 读取真实 CPU 使用率（psutil）
    """

    def __init__(self, node: Node):
        self.node = node

        # 声明真实硬件参数
        self.node.declare_parameter('real.battery_topic', '/battery')
        self.node.declare_parameter('real.battery_source', 'topic')  # 'topic', 'system', 'custom'
        self.node.declare_parameter('real.network_interface', 'wlan0')
        self.node.declare_parameter('real.base_station_ip', '192.168.1.1')
        self.node.declare_parameter('real.bandwidth_method', 'ping')  # 'ping', 'iperf', 'monitor'
        self.node.declare_parameter('real.bandwidth_test_interval', 5.0)

        # 读取参数
        self.battery_topic = self.node.get_parameter('real.battery_topic').value
        self.battery_source = self.node.get_parameter('real.battery_source').value
        self.network_interface = self.node.get_parameter('real.network_interface').value
        self.base_station_ip = self.node.get_parameter('real.base_station_ip').value
        self.bandwidth_method = self.node.get_parameter('real.bandwidth_method').value
        self.bandwidth_test_interval = self.node.get_parameter('real.bandwidth_test_interval').value

        # 状态变量
        self.current_battery = 1.0
        # 带宽单位统一为 Mbps
        self.current_bandwidth = 10.0
        self.last_bandwidth_test = time.time()

        # 订阅真实电池话题（如果使用 topic 模式）
        if self.battery_source == 'topic':
            self.battery_sub = self.node.create_subscription(
                BatteryState,
                self.battery_topic,
                self.battery_callback,
                10
            )

        self.node.get_logger().info('真实后端已初始化')

    def battery_callback(self, msg: BatteryState):
        """电池状态回调"""
        self.current_battery = msg.percentage

    def get_resource_state(self) -> Tuple[float, float, float]:
        """
        获取真实的资源状态
        
        Returns:
            (battery, bandwidth, cpu)
        """
        # 读取电池
        battery = self.read_battery()

        # 测量带宽
        bandwidth = self.measure_bandwidth()

        # 读取 CPU
        cpu = self.read_cpu()

        return battery, bandwidth, cpu

    def read_battery(self) -> float:
        """读取真实电池状态"""
        if self.battery_source == 'topic':
            # 从订阅的话题获取
            return self.current_battery

        elif self.battery_source == 'system':
            # 从系统文件读取（Linux）
            try:
                with open('/sys/class/power_supply/BAT0/capacity', 'r') as f:
                    capacity = int(f.read().strip())
                    return capacity / 100.0
            except Exception as e:
                self.node.get_logger().warning(f'无法读取系统电池: {e}')
                return 1.0

        else:
            # 自定义方法
            return 1.0

    def measure_bandwidth(self) -> float:
        """测量真实网络带宽"""
        current_time = time.time()

        # 检查是否需要重新测试
        if current_time - self.last_bandwidth_test < self.bandwidth_test_interval:
            return self.current_bandwidth

        if self.bandwidth_method == 'ping':
            bandwidth = self._measure_bandwidth_ping()
        elif self.bandwidth_method == 'iperf':
            bandwidth = self._measure_bandwidth_iperf()
        elif self.bandwidth_method == 'monitor':
            bandwidth = self._measure_bandwidth_monitor()
        else:
            bandwidth = 10.0

        self.current_bandwidth = bandwidth
        self.last_bandwidth_test = current_time

        return bandwidth

    def _measure_bandwidth_ping(self) -> float:
        """使用 ping 估算带宽"""
        try:
            import subprocess
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '1', self.base_station_ip],
                capture_output=True,
                timeout=2.0
            )

            if result.returncode == 0:
                # 解析延迟
                output = result.stdout.decode()
                if 'time=' in output:
                    time_str = output.split('time=')[1].split()[0]
                    latency = float(time_str)  # ms

                    # 根据延迟估算带宽（简化模型）
                    if latency < 10:
                        return 10.0
                    elif latency < 50:
                        return 6.0
                    elif latency < 100:
                        return 3.0
                    else:
                        return 1.0

            return 1.0  # ping 失败

        except Exception as e:
            self.node.get_logger().warning(f'Ping 测试失败: {e}')
            return 10.0

    def _measure_bandwidth_iperf(self) -> float:
        """使用 iperf3 测量带宽"""
        # TODO: 实现 iperf3 测试
        return 10.0

    def _measure_bandwidth_monitor(self) -> float:
        """监控网络接口流量"""
        # TODO: 实现网络流量监控
        return 10.0

    def read_cpu(self) -> float:
        """读取真实 CPU 使用率"""
        try:
            import psutil
            return psutil.cpu_percent(interval=0.1) / 100.0
        except ImportError:
            self.node.get_logger().warning('psutil 未安装，返回默认 CPU 值')
            return 0.3
        except Exception as e:
            self.node.get_logger().warning(f'无法读取 CPU: {e}')
            return 0.3

    def reset(self):
        """重置（真实后端不需要重置）"""
        pass
