#!/usr/bin/env python3
"""
dynamic_tf_broadcaster.py

动态TF广播器 - 根据机器人初始位置发布TF变换

功能:
1. 订阅 /robot_initial_poses 获取机器人位置
2. 发布 global_map -> robot_{id}/map 的静态TF
3. 位置更新时自动重新广播
4. 支持重置：收到重置信号后用最新位置重新广播TF

与原 map_tf_broadcaster.py 的区别:
- 动态订阅而非读取yaml配置文件
- 支持位置更新时重新广播
- 支持重置确认机制
"""

import math
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, PoseArray
from std_msgs.msg import Empty
from tf2_ros import StaticTransformBroadcaster
from cure_explore_interfaces.msg import ResetAck


class DynamicTFBroadcaster(LifecycleNode):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.get_logger().info('DynamicTFBroadcaster 节点已创建')

        # 内部状态
        self.tf_broadcaster = None
        self.poses_sub = None
        self.reset_sub = None
        self.reset_ack_pub = None
        self.current_poses = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """配置阶段"""
        self.get_logger().info('配置 DynamicTFBroadcaster...')

        # 声明参数
        self.declare_parameter('robot_count', 2)
        self.declare_parameter('global_frame', 'global_map')

        self.robot_count = self.get_parameter('robot_count').value
        self.global_frame = self.get_parameter('global_frame').value

        # 创建TF广播器
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # 订阅位置配置 (TRANSIENT_LOCAL 确保收到最新)
        qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )
        self.poses_sub = self.create_subscription(
            PoseArray, '/robot_initial_poses',
            self.poses_callback, qos)

        # 订阅重置信号
        self.reset_sub = self.create_subscription(
            Empty, '/cure_explore/reset',
            self.reset_callback, 10)

        # 发布重置确认
        self.reset_ack_pub = self.create_publisher(
            ResetAck, '/cure_explore/internal_reset_ack', 10)

        self.get_logger().info('配置完成，等待位置配置...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """激活阶段"""
        self.get_logger().info('激活 DynamicTFBroadcaster...')
        # 如果已经收到位置，立即发布
        if self.current_poses is not None:
            self._broadcast_transforms(self.current_poses)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """停用阶段"""
        self.get_logger().info('停用 DynamicTFBroadcaster...')
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """清理阶段"""
        self.get_logger().info('清理 DynamicTFBroadcaster...')
        self.current_poses = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """关闭阶段"""
        self.get_logger().info('关闭 DynamicTFBroadcaster...')
        return TransitionCallbackReturn.SUCCESS

    def poses_callback(self, msg: PoseArray):
        """接收位置配置并广播TF"""
        self.get_logger().info(f'收到位置配置: {len(msg.poses)} 个机器人')
        self.current_poses = msg
        self._broadcast_transforms(msg)

    def reset_callback(self, msg):
        """接收重置信号，使用当前位置重新广播TF"""
        self.get_logger().info('收到重置信号')

        success = True
        message = ''

        if self.current_poses is not None:
            # 使用当前缓存的位置重新广播TF
            self._broadcast_transforms(self.current_poses)
            message = f'TF广播器重置完成，重新广播了 {len(self.current_poses.poses)} 个TF'
            self.get_logger().info(message)
        else:
            # 没有位置配置，可能是配置顺序问题
            message = 'TF广播器重置：没有位置配置可用，等待新的位置配置'
            self.get_logger().warn(message)
            # 仍然视为成功，因为这可能是正常的初始状态

        # 发送重置确认
        ack = ResetAck()
        ack.source = 'dynamic_tf_broadcaster'
        ack.success = success
        ack.message = message
        self.reset_ack_pub.publish(ack)
        self.get_logger().info('已发送重置确认到 /cure_explore/internal_reset_ack')

    def _broadcast_transforms(self, pose_array: PoseArray):
        """广播所有机器人的TF变换"""
        transforms = []

        for i, pose in enumerate(pose_array.poses):
            if i >= self.robot_count:
                break

            robot_id = i

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.global_frame
            t.child_frame_id = f'robot_{robot_id}/map'

            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation

            # 计算yaw用于日志
            yaw = 2 * math.atan2(pose.orientation.z, pose.orientation.w)
            self.get_logger().info(
                f'广播 TF: {self.global_frame} -> robot_{robot_id}/map '
                f'(x={pose.position.x:.2f}, y={pose.position.y:.2f}, yaw={yaw:.2f})')

            transforms.append(t)

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
            self.get_logger().info(f'已广播 {len(transforms)} 个TF变换')
        else:
            self.get_logger().warn('没有TF变换可广播')


def main(args=None):
    rclpy.init(args=args)

    node = DynamicTFBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()