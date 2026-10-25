#!/usr/bin/env python3
import time
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray, Int32
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import OccupancyGrid
from frontier_msgs.msg import FrontierArrayExtended


class RLGlobalStateNode(Node):
    def __init__(self):
        super().__init__('rl_global_state_node')

        self.declare_parameter('robot_count', 3)
        self.declare_parameter('robot_namespace', 'robot_')
        self.declare_parameter('battery_topic_suffix', '/battery_state')
        self.declare_parameter('cpu_topic_suffix', '/cpu_state')
        self.declare_parameter('bandwidth_topic_suffix', '/network_state')
        self.declare_parameter('global_map_topic', '/global_map')
        self.declare_parameter('frontier_topic', '/frontiers_info')
        self.declare_parameter('conflict_topic', '/goal_select/conflict_count')
        self.declare_parameter('per_robot_state_topic', '/rl_training/per_robot_state')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('online_timeout_sec', 2.0)
        self.declare_parameter('summary_log_interval_sec', 5.0)
        self.declare_parameter('stale_warn_interval_sec', 10.0)
        self.declare_parameter('input_stale_sec', 3.0)
        self.declare_parameter('cpu_is_percent', False)
        self.declare_parameter('bandwidth_max', 10.0)
        self.declare_parameter('frontier_max', 50.0)
        self.declare_parameter('conflict_max', 2.0)
        self.declare_parameter('battery_avg_weight', 0.5)
        self.declare_parameter('battery_min_weight', 0.5)
        self.declare_parameter('coverage_use_roi', True)
        self.declare_parameter('coverage_roi_min_x', -15.0)
        self.declare_parameter('coverage_roi_max_x', 15.0)
        self.declare_parameter('coverage_roi_min_y', -15.0)
        self.declare_parameter('coverage_roi_max_y', 15.0)
        self.declare_parameter('coverage_total_override', 0)

        self.robot_count = int(self.get_parameter('robot_count').value)
        self.robot_namespace = str(self.get_parameter('robot_namespace').value)
        self.battery_suffix = str(self.get_parameter('battery_topic_suffix').value)
        self.cpu_suffix = str(self.get_parameter('cpu_topic_suffix').value)
        self.bandwidth_suffix = str(self.get_parameter('bandwidth_topic_suffix').value)
        self.map_topic = str(self.get_parameter('global_map_topic').value)
        self.frontier_topic = str(self.get_parameter('frontier_topic').value)
        self.conflict_topic = str(self.get_parameter('conflict_topic').value)
        self.per_robot_state_topic = str(self.get_parameter('per_robot_state_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.online_timeout = float(self.get_parameter('online_timeout_sec').value)
        self.summary_log_interval_sec = float(self.get_parameter('summary_log_interval_sec').value)
        self.stale_warn_interval_sec = float(self.get_parameter('stale_warn_interval_sec').value)
        self.input_stale_sec = float(self.get_parameter('input_stale_sec').value)
        self.cpu_is_percent = bool(self.get_parameter('cpu_is_percent').value)
        self.bandwidth_max = float(self.get_parameter('bandwidth_max').value)
        self.frontier_max = float(self.get_parameter('frontier_max').value)
        self.conflict_max = float(self.get_parameter('conflict_max').value)
        self.battery_avg_weight = float(self.get_parameter('battery_avg_weight').value)
        self.battery_min_weight = float(self.get_parameter('battery_min_weight').value)
        self.coverage_use_roi = bool(self.get_parameter('coverage_use_roi').value)
        self.coverage_roi_min_x = float(self.get_parameter('coverage_roi_min_x').value)
        self.coverage_roi_max_x = float(self.get_parameter('coverage_roi_max_x').value)
        self.coverage_roi_min_y = float(self.get_parameter('coverage_roi_min_y').value)
        self.coverage_roi_max_y = float(self.get_parameter('coverage_roi_max_y').value)
        self.coverage_total_override = int(self.get_parameter('coverage_total_override').value)

        self.last_battery: Dict[int, Optional[float]] = {i: None for i in range(self.robot_count)}
        self.last_cpu: Dict[int, Optional[float]] = {i: None for i in range(self.robot_count)}
        self.last_bandwidth: Dict[int, Optional[float]] = {i: None for i in range(self.robot_count)}
        self.last_update_time: Dict[int, float] = {i: 0.0 for i in range(self.robot_count)}

        self.coverage_ratio = 0.0
        self.frontier_count = 0
        self.last_conflict = 0
        now = time.time()
        self.last_map_time = now
        self.last_frontier_time = now
        self.last_conflict_time = now
        self.last_summary_log_time = 0.0
        self.last_stale_warn_time = 0.0

        # Subscribers
        for i in range(self.robot_count):
            robot_name = f'{self.robot_namespace}{i}'
            self.create_subscription(
                BatteryState,
                f'/{robot_name}{self.battery_suffix}',
                lambda msg, idx=i: self.battery_cb(idx, msg),
                10,
            )
            self.create_subscription(
                Float64,
                f'/{robot_name}{self.cpu_suffix}',
                lambda msg, idx=i: self.cpu_cb(idx, msg),
                10,
            )
            self.create_subscription(
                Float64,
                f'/{robot_name}{self.bandwidth_suffix}',
                lambda msg, idx=i: self.bandwidth_cb(idx, msg),
                10,
            )

        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            10,
        )

        self.create_subscription(
            FrontierArrayExtended,
            self.frontier_topic,
            self.frontier_cb,
            10,
        )

        self.create_subscription(
            Int32,
            self.conflict_topic,
            self.conflict_cb,
            10,
        )

        self.state_pub = self.create_publisher(Float64MultiArray, '/rl_training/global_state', 10)
        self.per_robot_state_pub = self.create_publisher(Float64MultiArray, self.per_robot_state_topic, 10)
        self.coverage_pub = self.create_publisher(Float64, '/rl_training/coverage_ratio', 10)

        self.timer = self.create_timer(1.0 / max(self.publish_rate, 0.1), self.publish_state)

        self.get_logger().info(
            f'RLGlobalStateNode started: robots={self.robot_count}, publish_rate={self.publish_rate:.2f}Hz, '
            f'summary_log_interval={self.summary_log_interval_sec:.1f}s, '
            f'coverage_use_roi={self.coverage_use_roi}, '
            f'roi=[{self.coverage_roi_min_x:.1f},{self.coverage_roi_max_x:.1f}]x'
            f'[{self.coverage_roi_min_y:.1f},{self.coverage_roi_max_y:.1f}]'
        )

    def battery_cb(self, idx: int, msg: BatteryState):
        self.last_battery[idx] = float(msg.percentage)
        self.last_update_time[idx] = time.time()

    def cpu_cb(self, idx: int, msg: Float64):
        self.last_cpu[idx] = float(msg.data)
        self.last_update_time[idx] = time.time()

    def bandwidth_cb(self, idx: int, msg: Float64):
        self.last_bandwidth[idx] = float(msg.data)
        self.last_update_time[idx] = time.time()

    def map_cb(self, msg: OccupancyGrid):
        if not msg.data:
            return
        self.last_map_time = time.time()
        data = np.asarray(msg.data, dtype=np.int16)

        total = data.size
        known = np.count_nonzero(data != -1)

        if self.coverage_use_roi:
            width = int(msg.info.width)
            height = int(msg.info.height)
            resolution = float(msg.info.resolution)
            origin_x = float(msg.info.origin.position.x)
            origin_y = float(msg.info.origin.position.y)

            if width > 0 and height > 0 and resolution > 0.0 and data.size == width * height:
                x0 = int(np.floor((self.coverage_roi_min_x - origin_x) / resolution))
                x1 = int(np.ceil((self.coverage_roi_max_x - origin_x) / resolution))
                y0 = int(np.floor((self.coverage_roi_min_y - origin_y) / resolution))
                y1 = int(np.ceil((self.coverage_roi_max_y - origin_y) / resolution))

                x0 = int(np.clip(x0, 0, width))
                x1 = int(np.clip(x1, 0, width))
                y0 = int(np.clip(y0, 0, height))
                y1 = int(np.clip(y1, 0, height))

                if x1 > x0 and y1 > y0:
                    grid = data.reshape(height, width)
                    roi = grid[y0:y1, x0:x1]
                    total = int(roi.size)
                    known = int(np.count_nonzero(roi != -1))

        if self.coverage_total_override > 0:
            total = int(self.coverage_total_override)

        if total > 0:
            self.coverage_ratio = min(1.0, float(known) / float(total))
        else:
            self.coverage_ratio = 0.0
        cov = Float64()
        cov.data = self.coverage_ratio
        self.coverage_pub.publish(cov)

    def frontier_cb(self, msg: FrontierArrayExtended):
        self.frontier_count = len(msg.frontiers_extended)
        self.last_frontier_time = time.time()

    def conflict_cb(self, msg: Int32):
        self.last_conflict = max(0, int(msg.data))
        self.last_conflict_time = time.time()

    def _normalize_cpu(self, value: float) -> float:
        if self.cpu_is_percent:
            return np.clip(value / 100.0, 0.0, 1.0)
        return np.clip(value, 0.0, 1.0)

    def publish_state(self):
        now = time.time()
        online = 0
        batteries = []
        cpus = []
        bandwidths = []
        for i in range(self.robot_count):
            if now - self.last_update_time[i] <= self.online_timeout:
                online += 1
            if self.last_battery[i] is not None:
                batteries.append(self.last_battery[i])
            if self.last_cpu[i] is not None:
                cpus.append(self.last_cpu[i])
            if self.last_bandwidth[i] is not None:
                bandwidths.append(self.last_bandwidth[i])

        avg_battery = float(np.mean(batteries)) if batteries else 1.0
        min_battery = float(np.min(batteries)) if batteries else 1.0
        battery_weight_sum = max(self.battery_avg_weight + self.battery_min_weight, 1e-6)
        battery_feature = (
            self.battery_avg_weight * avg_battery +
            self.battery_min_weight * min_battery
        ) / battery_weight_sum
        avg_cpu = float(np.mean([self._normalize_cpu(c) for c in cpus])) if cpus else 0.0

        if bandwidths:
            bw_norm = [b / max(self.bandwidth_max, 1e-6) for b in bandwidths]
            avg_bandwidth = float(np.clip(np.mean(bw_norm), 0.0, 1.0))
        else:
            avg_bandwidth = 1.0

        online_norm = float(online) / float(max(self.robot_count, 1))
        frontier_norm = float(self.frontier_count) / float(max(self.frontier_max, 1.0))
        conflict_norm = float(self.last_conflict) / float(max(self.conflict_max, 1.0))

        global_state = [
            np.clip(battery_feature, 0.0, 1.0),
            np.clip(avg_cpu, 0.0, 1.0),
            np.clip(avg_bandwidth, 0.0, 1.0),
            np.clip(self.coverage_ratio, 0.0, 1.0),
            np.clip(online_norm, 0.0, 1.0),
            np.clip(frontier_norm, 0.0, 1.0),
            np.clip(conflict_norm, 0.0, 1.0),
        ]

        msg = Float64MultiArray()
        msg.data = list(global_state)
        self.state_pub.publish(msg)

        # 每个机器人 4 维状态：[battery, cpu_norm, bandwidth_norm, online]
        per_robot_values = []
        for i in range(self.robot_count):
            battery_i = float(np.clip(self.last_battery[i], 0.0, 1.0)) if self.last_battery[i] is not None else 1.0
            cpu_i = float(self._normalize_cpu(self.last_cpu[i])) if self.last_cpu[i] is not None else 0.0
            if self.last_bandwidth[i] is not None:
                bw_i = float(np.clip(self.last_bandwidth[i] / max(self.bandwidth_max, 1e-6), 0.0, 1.0))
            else:
                bw_i = 1.0
            online_i = 1.0 if (now - self.last_update_time[i] <= self.online_timeout) else 0.0
            per_robot_values.extend([battery_i, cpu_i, bw_i, online_i])

        per_robot_msg = Float64MultiArray()
        per_robot_msg.data = per_robot_values
        self.per_robot_state_pub.publish(per_robot_msg)

        if now - self.last_summary_log_time >= self.summary_log_interval_sec:
            self.last_summary_log_time = now
            self.get_logger().info(
                f'[RL-STATE] global_state='
                f'[{global_state[0]:.3f}, {global_state[1]:.3f}, {global_state[2]:.3f}, '
                f'{global_state[3]:.3f}, {global_state[4]:.3f}, {global_state[5]:.3f}, {global_state[6]:.3f}] '
                f'raw(frontiers={self.frontier_count}, conflict={self.last_conflict}, '
                f'online={online}/{self.robot_count}, battery_avg={avg_battery:.3f}, battery_min={min_battery:.3f})'
            )

        # Stale-input warning (throttled)
        map_age = now - self.last_map_time
        frontier_age = now - self.last_frontier_time
        conflict_age = now - self.last_conflict_time
        if (
            map_age > self.input_stale_sec or
            frontier_age > self.input_stale_sec or
            conflict_age > self.input_stale_sec
        ) and (now - self.last_stale_warn_time >= self.stale_warn_interval_sec):
            self.last_stale_warn_time = now
            self.get_logger().warn(
                f'[RL-STATE] stale input detected: map_age={map_age:.1f}s, '
                f'frontier_age={frontier_age:.1f}s, conflict_age={conflict_age:.1f}s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RLGlobalStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
