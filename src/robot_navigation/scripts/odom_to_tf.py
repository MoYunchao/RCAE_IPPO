#!/usr/bin/env python3
"""
自动补偿的 Odometry to TF Broadcaster

自动检测frame_id是否为"world"，如果是则自动补偿到odom坐标系
无需任何额外参数配置
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from cure_explore_interfaces.msg import ResetCommand
from std_srvs.srv import Trigger
import numpy as np

class AutoCompensateOdomToTFNode(Node):
    def __init__(self):
        super().__init__('auto_compensate_odom_to_tf')
        
        # 声明基础参数（保持原样）
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('robot_id', 1)
        
        # 获取参数
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        robot_id = self.get_parameter('robot_id').value
        
        # 自动补偿相关
        self.initial_position = None  # 初始位置（世界坐标）
        self.initial_orientation = None  # 初始朝向
        self.needs_compensation = False  # 是否需要补偿
        self.is_active = True
        self.last_timestamp = None
        self.time_jump_threshold = 5.0
        self.backward_jump_threshold = 0.1
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        # 订阅重置命令
        reset_topic = f'/robot_{robot_id-1}/reset_command'
        self.reset_sub = self.create_subscription(
            ResetCommand,
            reset_topic,
            self.reset_callback,
            10
        )
        
        # 紧急清理服务客户端
        self.emergency_cleanup_client = self.create_client(
            Trigger,
            f'/robot_{robot_id-1}/emergency_cleanup_nav2'
        )
        
        self.get_logger().info(f'启动自动补偿TF广播器')
        self.get_logger().info(f'订阅话题: {odom_topic}')
    
    def reset_callback(self, msg: ResetCommand):
        """重置时清除初始位置记录"""
        if msg.command == 0:  # SHUTDOWN
            self.get_logger().info('重置：清除初始位置')
            self.initial_position = None
            self.initial_orientation = None
            self.needs_compensation = False
            self.is_active = False
        elif msg.command == 1:  # STARTUP
            self.is_active = True
        elif msg.command == 2:  # RESTART
            self.initial_position = None
            self.initial_orientation = None
            self.needs_compensation = False
    
    def odom_callback(self, msg):
        """自动检测并补偿世界坐标"""
        if not self.is_active:
            return
        
        # 时间跳跃检测（保持原逻辑）
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_timestamp is not None:
            time_diff = current_time - self.last_timestamp
            if time_diff < -self.backward_jump_threshold or time_diff > self.time_jump_threshold:
                self._handle_time_jump()
                return
        self.last_timestamp = current_time
        
        # 第一步：检测frame_id并决定是否需要补偿
        if self.initial_position is None:
            self._initialize_compensation(msg)
            return  # 第一次只记录，不广播
        
        # 第二步：应用补偿（如果需要）
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        if self.needs_compensation:
            # 应用坐标补偿：相对位置 = 当前世界位置 - 初始世界位置
            t.transform.translation.x = msg.pose.pose.position.x - self.initial_position[0]
            t.transform.translation.y = msg.pose.pose.position.y - self.initial_position[1]
            t.transform.translation.z = msg.pose.pose.position.z - self.initial_position[2]
            
            # 旋转补偿：相对旋转 = 当前旋转 * 初始旋转的逆
            # 简化为直接使用当前旋转（对于大多数地面机器人足够）
            t.transform.rotation = msg.pose.pose.orientation
        else:
            # 直接使用原始数据（已经是相对坐标）
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
        
        # 广播TF变换
        self.tf_broadcaster.sendTransform(t)
    
    def _initialize_compensation(self, msg):
        """初始化补偿参数"""
        # 检查frame_id是否为"world"
        if msg.header.frame_id.lower() in ['world', 'map', 'world_frame']:
            self.needs_compensation = True
            self.get_logger().info(f'检测到frame_id="{msg.header.frame_id}"，启用自动补偿')
        else:
            self.needs_compensation = False
            self.get_logger().info(f'检测到frame_id="{msg.header.frame_id}"，使用原始数据')
        
        # 记录初始位置（无论是否需要补偿）
        self.initial_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        
        self.initial_orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        
        self.get_logger().info(f'初始位置记录: {self.initial_position}')
        
        # 如果启用了补偿，打印补偿信息
        if self.needs_compensation:
            self.get_logger().info('补偿模式：将世界坐标转换为相对坐标')
            self.get_logger().info(f'odom坐标系原点将固定在: {self.initial_position}')
    
    def _handle_time_jump(self):
        """处理时间跳跃"""
        self.get_logger().warn('检测到时间跳跃，重置补偿参数')
        self.initial_position = None
        self.initial_orientation = None
        self.needs_compensation = False
        self._emergency_kill_slam()
    
    def _emergency_kill_slam(self):
        """紧急清理"""
        if self.emergency_cleanup_client.service_is_ready():
            try:
                request = Trigger.Request()
                future = self.emergency_cleanup_client.call_async(request)
                self.get_logger().info('已调用紧急清理服务')
            except Exception as e:
                self.get_logger().error(f'调用紧急清理服务失败: {e}')
        else:
            self.get_logger().warn('紧急清理服务未就绪')

def main(args=None):
    rclpy.init(args=args)
    node = AutoCompensateOdomToTFNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
