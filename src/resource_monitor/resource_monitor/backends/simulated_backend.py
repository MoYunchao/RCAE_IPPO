#!/usr/bin/env python3
"""
Resource Backends - 资源监控后端

包含：
- SimulatedBackend: 模拟后端（训练时使用）
- RealBackend: 真实后端（推理时使用）

作者：RL Training System
日期：2026-01-28
"""

import numpy as np
import time
from typing import Tuple
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState


class SimulatedBackend:
    """
    模拟后端（训练时使用）
    
    功能：
    - 基于时间和移动距离模拟电量消耗
    - 基于距离模拟带宽变化
    - 模拟 CPU 使用率
    """
    
    def __init__(self, node: Node):
        self.node = node
        
        # 声明模拟参数
        self.node.declare_parameter('sim.initial_battery', 1.0)
        self.node.declare_parameter('sim.base_consumption', 0.0004)
        self.node.declare_parameter('sim.movement_consumption', 0.005)
        self.node.declare_parameter('sim.sensor_consumption', 0.0002)
        # 带宽单位统一为 Mbps
        self.node.declare_parameter('sim.max_bandwidth', 10.0)
        self.node.declare_parameter('sim.min_bandwidth', 1.0)
        self.node.declare_parameter('sim.bandwidth_decay_rate', 0.35)
        self.node.declare_parameter('sim.base_station_x', 0.0)
        self.node.declare_parameter('sim.base_station_y', 0.0)
        self.node.declare_parameter('sim.battery_noise', 0.0002)
        self.node.declare_parameter('sim.bandwidth_noise', 0.2)
        # CPU 模型（输出 0~1）
        self.node.declare_parameter('sim.cpu_idle', 0.25)
        self.node.declare_parameter('sim.cpu_speed_ref', 1.0)
        self.node.declare_parameter('sim.cpu_speed_gain', 0.50)
        self.node.declare_parameter('sim.cpu_cmd_linear_gain', 0.12)
        self.node.declare_parameter('sim.cpu_cmd_angular_gain', 0.08)
        self.node.declare_parameter('sim.cpu_noise', 0.03)
        
        # 读取参数
        self.battery = self.node.get_parameter('sim.initial_battery').value
        self.base_consumption = self.node.get_parameter('sim.base_consumption').value
        self.movement_consumption = self.node.get_parameter('sim.movement_consumption').value
        self.sensor_consumption = self.node.get_parameter('sim.sensor_consumption').value
        self.max_bandwidth = self.node.get_parameter('sim.max_bandwidth').value
        self.min_bandwidth = self.node.get_parameter('sim.min_bandwidth').value
        self.bandwidth_decay_rate = self.node.get_parameter('sim.bandwidth_decay_rate').value
        self.base_station_position = np.array([
            self.node.get_parameter('sim.base_station_x').value,
            self.node.get_parameter('sim.base_station_y').value
        ])
        self.battery_noise = self.node.get_parameter('sim.battery_noise').value
        self.bandwidth_noise = self.node.get_parameter('sim.bandwidth_noise').value
        self.cpu_idle = self.node.get_parameter('sim.cpu_idle').value
        self.cpu_speed_ref = max(self.node.get_parameter('sim.cpu_speed_ref').value, 1e-3)
        self.cpu_speed_gain = self.node.get_parameter('sim.cpu_speed_gain').value
        self.cpu_cmd_linear_gain = self.node.get_parameter('sim.cpu_cmd_linear_gain').value
        self.cpu_cmd_angular_gain = self.node.get_parameter('sim.cpu_cmd_angular_gain').value
        self.cpu_noise = self.node.get_parameter('sim.cpu_noise').value
        
        # 状态变量
        self.last_position = None
        self.last_time = time.time()
        self.current_position = np.array([0.0, 0.0])
        self.current_speed_mps = 0.0
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        
        # 订阅 odom（用于计算移动距离）
        robot_id = self.node.get_parameter('robot_id').value
        robot_namespace = self.node.get_parameter('robot_namespace').value
        robot_name = f"{robot_namespace}{robot_id}"
        
        self.odom_sub = self.node.create_subscription(
            Odometry,
            f'/{robot_name}/odom',
            self.odom_callback,
            10
        )

        # 订阅 cmd_vel（用于模拟 CPU 负载变化）
        self.cmd_vel_sub = self.node.create_subscription(
            Twist,
            f'/{robot_name}/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.node.get_logger().info('模拟后端已初始化')
    
    def odom_callback(self, msg: Odometry):
        """Odometry 回调"""
        # 更新当前位置
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        
        # 初始化上一次位置
        if self.last_position is None:
            self.last_position = self.current_position.copy()

    def cmd_vel_callback(self, msg: Twist):
        """cmd_vel 回调"""
        self.last_cmd_linear = float(np.hypot(msg.linear.x, msg.linear.y))
        self.last_cmd_angular = float(abs(msg.angular.z))
    
    def get_resource_state(self) -> Tuple[float, float, float]:
        """
        获取模拟的资源状态
        
        Returns:
            (battery, bandwidth, cpu)
        """
        # 更新电量
        self.update_battery()
        
        # 更新带宽
        bandwidth = self.update_bandwidth()
        
        # 模拟 CPU（与运动强度相关）
        cpu = self.update_cpu()
        
        return self.battery, bandwidth, cpu
    
    def update_battery(self):
        """更新电量"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        # 计算移动距离
        if self.last_position is not None:
            distance = np.linalg.norm(self.current_position - self.last_position)
        else:
            distance = 0.0

        # 估计当前线速度（m/s）
        if dt > 1e-3:
            self.current_speed_mps = float(distance / dt)
        else:
            self.current_speed_mps = 0.0
        
        # 基础消耗
        base_cost = self.base_consumption * dt
        
        # 移动消耗
        movement_cost = self.movement_consumption * distance
        
        # 传感器消耗
        sensor_cost = self.sensor_consumption * dt
        
        # 总消耗
        total_cost = base_cost + movement_cost + sensor_cost
        
        # 添加噪声
        noise = np.random.normal(0, self.battery_noise)
        total_cost += abs(noise)  # 只能消耗，不能增加
        
        # 更新电量
        self.battery -= total_cost
        self.battery = np.clip(self.battery, 0.0, 1.0)
        
        # 更新状态
        self.last_position = self.current_position.copy()
        self.last_time = current_time
    
    def update_bandwidth(self) -> float:
        """更新带宽（Mbps）"""
        # 计算到基站的距离
        distance = np.linalg.norm(self.current_position - self.base_station_position)
        
        # 基于距离的带宽衰减
        bandwidth = self.max_bandwidth - self.bandwidth_decay_rate * distance
        bandwidth = np.clip(bandwidth, self.min_bandwidth, self.max_bandwidth)
        
        # 添加随机波动
        noise = np.random.normal(0, self.bandwidth_noise)
        bandwidth += noise
        bandwidth = np.clip(bandwidth, self.min_bandwidth, self.max_bandwidth)
        
        return bandwidth

    def update_cpu(self) -> float:
        """更新 CPU 占用率（0~1）"""
        speed_norm = min(self.current_speed_mps / self.cpu_speed_ref, 2.0)
        cpu = (
            self.cpu_idle
            + self.cpu_speed_gain * speed_norm
            + self.cpu_cmd_linear_gain * self.last_cmd_linear
            + self.cpu_cmd_angular_gain * self.last_cmd_angular
        )
        cpu += np.random.normal(0, self.cpu_noise)
        return float(np.clip(cpu, 0.0, 1.0))

    def get_current_position(self):
        """返回当前估计位置 (x, y)"""
        return self.current_position.copy()

    def estimate_return_energy(self, distance: float, speed_mps: float) -> float:
        """
        估算返回基站所需电量
        distance: 米
        speed_mps: m/s
        """
        if speed_mps <= 1e-3:
            speed_mps = 0.1
        travel_time = distance / speed_mps
        move_cost = self.movement_consumption * distance
        time_cost = (self.base_consumption + self.sensor_consumption) * travel_time
        return float(move_cost + time_cost)
    
    def reset(self):
        """重置资源状态"""
        self.battery = self.node.get_parameter('sim.initial_battery').value
        self.last_position = None
        self.last_time = time.time()
        self.node.get_logger().info('模拟后端已重置')


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
        self.current_bandwidth = 500.0
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
            bandwidth = 500.0
        
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
                        return 500.0
                    elif latency < 50:
                        return 300.0
                    elif latency < 100:
                        return 150.0
                    else:
                        return 50.0
            
            return 50.0  # ping 失败
        
        except Exception as e:
            self.node.get_logger().warning(f'Ping 测试失败: {e}')
            return 500.0
    
    def _measure_bandwidth_iperf(self) -> float:
        """使用 iperf3 测量带宽"""
        # TODO: 实现 iperf3 测试
        return 500.0
    
    def _measure_bandwidth_monitor(self) -> float:
        """监控网络接口流量"""
        # TODO: 实现网络流量监控
        return 500.0
    
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
