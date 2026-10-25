#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from cure_explore_interfaces.msg import ResetAck

from semantic_grid_msgs.msg import SemanticGrid


class MultiRobotSemanticMapper(Node):
    def __init__(self):
        super().__init__('multi_robot_semantic_mapper')

        # ---------------- robots ----------------
        self.robot_ids = ['robot_0', 'robot_1', 'robot_2']

        # ---------------- map storage ----------------
        self.global_map = None
        self.semantic_value_map = None   # float32, flattened  (语义价值)
        # ADDED: 语义风险地图
        self.semantic_risk_map = None    # float32, flattened  (语义风险)

        self.current_score = {}   # 语义价值
        # ADDED: 存储每个机器人最新的语义风险
        self.current_risk = {}

        self.ALPHA = 0.4   # EMA 融合系数

        # ---------------- debug mapping params ----------------
        self.DEBUG_MIN = -25.0
        self.DEBUG_MAX = 45.0

        # ---------------- subscribers ----------------
        self.create_subscription(
            OccupancyGrid,
            '/global_map',
            self.global_map_cb,
            1
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        for rid in self.robot_ids:
            # 订阅局部占据栅格地图
            self.create_subscription(
                OccupancyGrid,
                f'/{rid}/local_grid',
                lambda msg, rid=rid: self.local_grid_cb(msg, rid),
                10
            )
            # 订阅语义价值
            self.create_subscription(
                Float32,
                f'/{rid}/clip_semantic_score',
                lambda msg, rid=rid: self.semantic_score_cb(msg, rid),
                10
            )
            # ADDED: 订阅语义风险
            self.create_subscription(
                Float32,
                f'/{rid}/clip_semantic_risk',
                lambda msg, rid=rid: self.semantic_risk_cb(msg, rid),
                10
            )

        # ---------------- publishers ----------------
        # 原有语义价值地图发布
        self.value_pub = self.create_publisher(
            SemanticGrid,
            '/semantic_grid_float',
            1
        )
        # ADDED: 语义风险地图发布
        self.risk_pub = self.create_publisher(
            SemanticGrid,
            '/semantic_risk_grid_float',
            1
        )
        self.reset_ack_pub = self.create_publisher(
            ResetAck,
            '/cure_explore/internal_reset_ack',
            10
        )

        self.create_subscription(
            Empty,
            '/cure_explore/reset',
            self.global_reset_cb,
            10
        )

        # 调试地图发布（仅语义价值，保持不变）
        self.debug_pub = self.create_publisher(
            OccupancyGrid,
            '/semantic_grid_debug',
            1
        )

        self.get_logger().info(
            "语义地图节点启动：同时构建语义价值地图和语义风险地图"
        )

    def global_reset_cb(self, msg):
        del msg
        ok = True
        ack_msg = 'semantic maps cleared'
        try:
            # 先清空状态，再立即 ACK；重型发布放在后面，避免 ACK 超时
            self.reset_state(publish_now=False)
        except Exception as exc:
            ok = False
            ack_msg = f'semantic reset exception: {exc}'
            self.get_logger().error(f'语义地图重置异常: {exc}')

        ack = ResetAck()
        ack.source = 'semantic_mapper'
        ack.success = ok
        ack.message = ack_msg
        self.reset_ack_pub.publish(ack)

        if ok:
            try:
                self.publish_maps()
            except Exception as exc:
                self.get_logger().warn(f'语义地图重置后发布失败: {exc}')
            self.get_logger().info('收到 /cure_explore/reset，语义地图已重置')

    def reset_state(self, publish_now=True):
        if self.global_map is not None:
            size = self.global_map.info.width * self.global_map.info.height
            self.semantic_value_map = np.full(size, np.nan, dtype=np.float32)
            self.semantic_risk_map = np.full(size, np.nan, dtype=np.float32)
        else:
            self.semantic_value_map = None
            self.semantic_risk_map = None

        self.current_score.clear()
        self.current_risk.clear()
        if publish_now:
            self.publish_maps()

    # ==========================================================
    def global_map_cb(self, msg):
        self.global_map = msg
        size = msg.info.width * msg.info.height

        if self.semantic_value_map is None:
            self.semantic_value_map = np.full(size, np.nan, dtype=np.float32)
        # ADDED: 初始化语义风险地图
        if self.semantic_risk_map is None:
            self.semantic_risk_map = np.full(size, np.nan, dtype=np.float32)

    # ==========================================================
    def semantic_score_cb(self, msg, robot_id):
        """语义价值回调"""
        self.current_score[robot_id] = msg.data

    # ==========================================================
    # ADDED: 语义风险回调
    def semantic_risk_cb(self, msg, robot_id):
        self.current_risk[robot_id] = msg.data

    # ==========================================================
    def local_grid_cb(self, grid, robot_id):
        if self.global_map is None:
            return
        # 需要同时有价值和风险数据才处理，若缺失则跳过（可根据需求调整）
        if robot_id not in self.current_score or robot_id not in self.current_risk:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_map.header.frame_id,
                grid.header.frame_id,
                rclpy.time.Time()
            )
        except Exception:
            return

        score = self.current_score[robot_id]
        risk = self.current_risk[robot_id]   # ADDED

        local = np.array(grid.data).reshape(
            grid.info.height, grid.info.width
        )

        for y in range(grid.info.height):
            for x in range(grid.info.width):
                if local[y, x] != 0:
                    continue   # 只处理空闲栅格

                lx = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution
                ly = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution
                gx, gy = self.transform_point(lx, ly, tf)

                mx = int((gx - self.global_map.info.origin.position.x)
                         / self.global_map.info.resolution)
                my = int((gy - self.global_map.info.origin.position.y)
                         / self.global_map.info.resolution)

                if 0 <= mx < self.global_map.info.width and \
                   0 <= my < self.global_map.info.height:
                    # 同时更新语义价值和风险
                    self.update_cell(mx, my, score, risk)

        self.publish_maps()

    # ==========================================================
    # MODIFIED: 同时更新两张地图
    def update_cell(self, mx, my, score, risk):
        idx = my * self.global_map.info.width + mx

        # 更新语义价值
        old_score = self.semantic_value_map[idx]
        if np.isnan(old_score):
            self.semantic_value_map[idx] = score
        else:
            self.semantic_value_map[idx] = (
                self.ALPHA * score + (1.0 - self.ALPHA) * old_score
            )

        # 更新语义风险
        old_risk = self.semantic_risk_map[idx]
        if np.isnan(old_risk):
            self.semantic_risk_map[idx] = risk
        else:
            self.semantic_risk_map[idx] = (
                self.ALPHA * risk + (1.0 - self.ALPHA) * old_risk
            )

    # ==========================================================
    def transform_point(self, x, y, tf):
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        q = tf.transform.rotation
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        gx = math.cos(yaw) * x - math.sin(yaw) * y + tx
        gy = math.sin(yaw) * x + math.cos(yaw) * y + ty
        return gx, gy

    # ==========================================================
    def publish_maps(self):
        if (
            self.global_map is None or
            self.semantic_value_map is None or
            self.semantic_risk_map is None
        ):
            return

        now = self.get_clock().now().to_msg()
        frame = self.global_map.header.frame_id

        # ---------- 语义价值地图 (float) ----------
        value_msg = SemanticGrid()
        value_msg.header.stamp = now
        value_msg.header.frame_id = frame
        value_msg.info = self.global_map.info
        value_msg.data = self.semantic_value_map.astype(np.float32).tolist()
        self.value_pub.publish(value_msg)

        # ---------- 语义风险地图 (float) ----------
        # ADDED
        risk_msg = SemanticGrid()
        risk_msg.header.stamp = now
        risk_msg.header.frame_id = frame
        risk_msg.info = self.global_map.info
        risk_msg.data = self.semantic_risk_map.astype(np.float32).tolist()
        self.risk_pub.publish(risk_msg)

        # ---------- 调试地图 (仅语义价值，可不变) ----------
        dbg = OccupancyGrid()
        dbg.header = value_msg.header
        dbg.info = self.global_map.info

        vis = self.semantic_value_map.copy()
        vis = np.nan_to_num(vis, nan=self.DEBUG_MIN)
        vis = np.clip(vis, self.DEBUG_MIN, self.DEBUG_MAX)
        vis = (vis - self.DEBUG_MIN) / (self.DEBUG_MAX - self.DEBUG_MIN)
        vis = (vis * 100.0).astype(np.int8)

        dbg.data = vis.tolist()
        self.debug_pub.publish(dbg)


# ==============================================================
def main():
    rclpy.init()
    node = MultiRobotSemanticMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
