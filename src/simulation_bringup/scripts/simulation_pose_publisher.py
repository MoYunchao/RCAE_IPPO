#!/usr/bin/env python3
"""
仿真域初始位姿发布节点

从配置文件读取机器人初始位姿，发布到 /robot_initial_poses 话题。
该话题会通过 pose_relay.yaml 转发到服务器域和各机器人域。
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
import math


class SimulationPosePublisher(Node):
    def __init__(self):
        super().__init__('simulation_pose_publisher')
        
        # 声明参数
        self.declare_parameter('publish_initial_poses', True)
        self.declare_parameter('robot_count', 3)
        self.declare_parameter('poses_config_file', '')
        
        self.publish_enabled = self.get_parameter('publish_initial_poses').value
        self.robot_count = self.get_parameter('robot_count').value
        self.poses_config_file = str(self.get_parameter('poses_config_file').value)
        
        if not self.publish_enabled:
            self.get_logger().info('初始位姿发布已禁用 (publish_initial_poses=False)')
            return
        
        # 读取配置文件
        self.poses = self._load_robot_poses()
        
        # 创建 TRANSIENT_LOCAL QoS 发布器
        latching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher = self.create_publisher(
            PoseArray,
            '/robot_initial_poses',
            latching_qos
        )
        
        # 构建并发布位姿数组
        self._publish_poses()
        
        self.get_logger().info(f'仿真域初始位姿发布完成 ({len(self.poses)} 个机器人)')
    
    def _load_robot_poses(self):
        """从配置文件加载机器人初始位姿"""
        try:
            if self.poses_config_file:
                config_file = self.poses_config_file
            else:
                config_file = os.path.join(
                    get_package_share_directory("simulation_bringup"),
                    "config",
                    "robot_initial_poses.yaml"
                )
            
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            poses = config.get('robot_poses', {})
            self.get_logger().info(f'从 {config_file} 加载了 {len(poses)} 个机器人位姿')
            return poses
            
        except Exception as e:
            self.get_logger().error(f'加载配置文件失败: {e}')
            return {}
    
    def _publish_poses(self):
        """发布位姿数组"""
        pose_array = PoseArray()
        pose_array.header.frame_id = 'global_map'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(self.robot_count):
            robot_name = f'robot_{i}'
            pose_data = self.poses.get(robot_name, {})
            
            x = pose_data.get('x', 0.0)
            y = pose_data.get('y', 0.0)
            z = pose_data.get('z', 0.0)
            yaw = pose_data.get('yaw', 0.0)
            
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            
            # 从yaw角计算四元数 (绕Z轴旋转)
            half_yaw = yaw * 0.5
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin(half_yaw)
            pose.orientation.w = math.cos(half_yaw)
            
            pose_array.poses.append(pose)
            
            self.get_logger().info(
                f'添加 {robot_name}: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}'
            )
        
        self.publisher.publish(pose_array)
        self.get_logger().info('已发布初始位姿到 /robot_initial_poses')


def main(args=None):
    rclpy.init(args=args)
    node = SimulationPosePublisher()
    
    try:
        # 发布一次后保持运行以维持 TRANSIENT_LOCAL 消息
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
