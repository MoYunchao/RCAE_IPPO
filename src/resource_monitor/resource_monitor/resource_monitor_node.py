#!/usr/bin/env python3
"""
Resource Monitor - 资源监控节点

功能：
- 训练时：使用模拟后端
- 推理时：使用真实后端
- 统一发布资源状态

作者：RL Training System
日期：2026-01-28
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Empty
from cure_explore_interfaces.msg import ResetAck
from resource_monitor.backends import SimulatedBackend


class ResourceMonitor(Node):
    """
    资源监控节点（通用）
    
    功能：
    - 根据模式选择后端（模拟/真实）
    - 统一发布资源状态
    - 提供重置服务
    
    参数：
    - mode: 固定为 simulation（兼容参数保留）
    - robot_id: 机器人 ID
    - robot_namespace: 机器人命名空间
    - update_rate: 更新频率（Hz）
    """
    
    def __init__(self):
        super().__init__('resource_monitor')
        
        # 声明参数
        self.declare_parameter('mode', 'simulation')  # 兼容保留，固定使用 simulation
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('robot_namespace', 'robot_')
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('publish_battery', False)
        self.declare_parameter('publish_cpu', True)
        self.declare_parameter('publish_bandwidth', True)
        self.declare_parameter('low_battery_threshold', 0.2)
        self.declare_parameter('recharge_cooldown_sec', 5.0)
        self.declare_parameter('publish_low_battery_count', True)
        
        # 读取参数
        requested_mode = str(self.get_parameter('mode').value)
        mode = 'simulation'
        robot_id = self.get_parameter('robot_id').value
        robot_namespace = self.get_parameter('robot_namespace').value
        update_rate = self.get_parameter('update_rate').value
        publish_battery_requested = bool(self.get_parameter('publish_battery').value)
        self.publish_cpu_enabled = bool(self.get_parameter('publish_cpu').value)
        self.publish_bandwidth_enabled = bool(self.get_parameter('publish_bandwidth').value)
        self.publish_battery_enabled = False

        if publish_battery_requested:
            self.get_logger().warn(
                'resource_monitor 的 battery 发布已禁用；请使用 battery_state_sim 的 /battery_state'
            )
        
        # 固定为 simulation，避免误接真实资源接口
        if requested_mode != 'simulation':
            self.get_logger().warn(
                f"resource_monitor 当前固定 simulation 模式，忽略请求 mode='{requested_mode}'"
            )
        self.backend = SimulatedBackend(self)
        self.get_logger().info(f'使用模拟后端（机器人 {robot_id}）')
        
        # 创建发布者（统一接口）
        robot_name = f'{robot_namespace}{robot_id}'
        self.robot_name = robot_name
        
        self.bandwidth_pub = self.create_publisher(
            Float64,
            f'/{robot_name}/network_state',
            10
        )
        
        self.cpu_pub = self.create_publisher(
            Float64,
            f'/{robot_name}/cpu_state',
            10
        )

        self.reset_ack_pub = self.create_publisher(
            ResetAck,
            '/cure_explore/internal_reset_ack',
            10
        )

        self.global_reset_sub = self.create_subscription(
            Empty,
            '/cure_explore/reset',
            self.handle_global_reset,
            10
        )

        
        # 创建定时器
        self.timer = self.create_timer(1.0 / update_rate, self.update_callback)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'资源监控节点已启动')
        self.get_logger().info(f'模式: {mode}')
        self.get_logger().info(f'机器人: {robot_name}')
        self.get_logger().info(f'更新频率: {update_rate} Hz')
        self.get_logger().info(
            f'发布开关: battery={self.publish_battery_enabled}, '
            f'bandwidth={self.publish_bandwidth_enabled}, cpu={self.publish_cpu_enabled}'
        )
        self.get_logger().info('=' * 60)
    
    def update_callback(self):
        """定时更新资源状态"""
        try:
            # 从后端获取资源状态
            _, bandwidth, cpu = self.backend.get_resource_state()
            
            # 发布资源状态
            if self.publish_bandwidth_enabled:
                self.publish_bandwidth(bandwidth)
            if self.publish_cpu_enabled:
                self.publish_cpu(cpu)
            
            # 调试输出
            self.get_logger().debug(
                f'资源状态: 带宽={bandwidth:.2f}Mbps, CPU={cpu*100:.1f}%'
            )
        
        except Exception as e:
            self.get_logger().error(f'更新资源状态失败: {e}')
    
    def publish_bandwidth(self, bandwidth: float):
        """发布带宽状态"""
        msg = Float64()
        msg.data = bandwidth
        self.bandwidth_pub.publish(msg)
    
    def publish_cpu(self, cpu: float):
        """发布 CPU 状态"""
        msg = Float64()
        msg.data = cpu
        self.cpu_pub.publish(msg)
    
    def reset(self):
        """重置资源状态"""
        self.backend.reset()
        self.get_logger().info('资源状态已重置')

    def handle_global_reset(self, msg):
        del msg
        self.reset()
        ack = ResetAck()
        ack.source = f'{self.robot_name}/resource_monitor'
        ack.success = True
        ack.message = 'resource state reset'
        self.reset_ack_pub.publish(ack)
        self.get_logger().info('收到 /cure_explore/reset，资源状态已重置')

def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
