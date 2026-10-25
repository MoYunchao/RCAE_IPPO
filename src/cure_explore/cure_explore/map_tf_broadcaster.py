"""
发布 global_map -> robot_{id}/map 的静态 TF 变换。

来源：multi_robot_explore/config/robot_offsets.yaml
每个机器人的 offset_x, offset_y, offset_theta（弧度，逆时针）
"""

import os
import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory


class MapTFBroadcaster(Node):
    def __init__(self):
        super().__init__('map_tf_broadcaster')
        # 参数
        self.declare_parameter('robot_count', 3)
        self.declare_parameter('global_frame', 'global_map')
        self.robot_count = int(self.get_parameter('robot_count').value)
        self.global_frame = str(self.get_parameter('global_frame').value)

        # 读取配置
        self.offsets = self._load_offsets()

        # 静态 TF 广播器
        self.broadcaster = StaticTransformBroadcaster(self)

        # 发布所有机器人的静态变换
        transforms = []
        for i in range(1, self.robot_count + 1):
            off = self.offsets.get(f'robot_{i}', {})
            x = float(off.get('offset_x', 0.0))
            y = float(off.get('offset_y', 0.0))
            theta = float(off.get('offset_theta', 0.0))  # 弧度，逆时针

            t = TransformStamped()
            t.header.frame_id = self.global_frame
            t.child_frame_id = f'robot_{i}/map'
            # 平移
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            # 旋转（绕 Z 轴）
            half = theta * 0.5
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(half)
            t.transform.rotation.w = math.cos(half)

            transforms.append(t)
            self.get_logger().info(
                f'Broadcast static TF: {self.global_frame} -> robot_{i}/map (x={x}, y={y}, yaw={theta} rad)'
            )

        if transforms:
            self.broadcaster.sendTransform(transforms)
        else:
            self.get_logger().warn('No transforms to broadcast (robot_count=0 or missing offsets).')

    def _load_offsets(self):
        try:
            pkg_share = get_package_share_directory('cure_explore')
            config_path = os.path.join(pkg_share, 'config', 'robot_offsets.yaml')
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            return data
        except Exception as e:
            self.get_logger().error(f'Failed to load offsets: {e}')
            return {}


def main(args=None):
    rclpy.init(args=args)
    node = MapTFBroadcaster()
    try:
        # 作为 static broadcaster，保持进程运行即可
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
