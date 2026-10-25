#!/usr/bin/env python3
import os
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .rl_policy import build_policy


class RLWeightNode(Node):
    def __init__(self):
        super().__init__('rl_weight_node')

        self.declare_parameter('global_state_topic', '/rl_training/global_state')
        self.declare_parameter('weight_topic', '/rl_training/weights')
        self.declare_parameter('mode', 'infer')  # infer | train (train reserved)
        self.declare_parameter('model_path', '')
        self.declare_parameter('hidden_dim', 64)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('temperature', 1.0)
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('smoothing', 0.3)
        self.declare_parameter('log_interval_sec', 2.0)
        self.declare_parameter('no_state_warn_interval_sec', 5.0)
        self.declare_parameter('stale_state_warn_sec', 3.0)

        self.global_state_topic = str(self.get_parameter('global_state_topic').value)
        self.weight_topic = str(self.get_parameter('weight_topic').value)
        self.mode = str(self.get_parameter('mode').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.hidden_dim = int(self.get_parameter('hidden_dim').value)
        self.device = str(self.get_parameter('device').value)
        self.temperature = float(self.get_parameter('temperature').value)
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.smoothing = float(self.get_parameter('smoothing').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)
        self.no_state_warn_interval_sec = float(self.get_parameter('no_state_warn_interval_sec').value)
        self.stale_state_warn_sec = float(self.get_parameter('stale_state_warn_sec').value)

        self.last_state: Optional[np.ndarray] = None
        self.last_weights = np.array([0.25, 0.25, 0.25, 0.25], dtype=np.float32)
        self.last_state_time = 0.0
        self.last_log_time = 0.0
        self.last_no_state_warn_time = 0.0
        self.last_stale_warn_time = 0.0

        if not self.model_path:
            raise RuntimeError('model_path is required for RLWeightNode (no heuristic fallback).')

        try:
            self.policy = build_policy(
                state_dim=7,
                action_dim=4,
                hidden_dim=self.hidden_dim,
                device=self.device,
            )
        except Exception as e:
            raise RuntimeError(f'Failed to build RL policy: {e}') from e

        loaded = self.policy.load(self.model_path)
        if loaded:
            self.get_logger().info(f'Loaded RL model: {self.model_path}')
        else:
            raise RuntimeError(f'RL model not found or failed to load: {self.model_path}')

        self.create_subscription(
            Float64MultiArray,
            self.global_state_topic,
            self.global_state_cb,
            10,
        )

        self.weight_pub = self.create_publisher(Float64MultiArray, self.weight_topic, 10)

        self.timer = self.create_timer(1.0 / max(self.update_rate, 0.1), self.step)

        self.get_logger().info(
            f'RLWeightNode started: mode={self.mode}, topic={self.weight_topic}, '
            f'rate={self.update_rate:.2f}Hz, log_interval={self.log_interval_sec:.1f}s'
        )

    def global_state_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 7:
            self.get_logger().warn(
                f'Ignored invalid global_state length={len(msg.data)} (expected >=7)'
            )
            return
        state = np.array(msg.data[:7], dtype=np.float32)
        if not np.all(np.isfinite(state)):
            self.get_logger().warn(f'Ignored non-finite global_state: {state.tolist()}')
            return
        self.last_state = state
        self.last_state_time = time.time()

    def step(self):
        now = time.time()
        if self.last_state is None:
            if now - self.last_no_state_warn_time >= self.no_state_warn_interval_sec:
                self.last_no_state_warn_time = now
                self.get_logger().warn(
                    f'No global_state received yet on {self.global_state_topic}, skipping weight publish'
                )
            return
        state_age = now - self.last_state_time if self.last_state_time > 0 else 0.0
        if state_age > self.stale_state_warn_sec and now - self.last_stale_warn_time >= self.no_state_warn_interval_sec:
            self.last_stale_warn_time = now
            self.get_logger().warn(
                f'Global state is stale (age={state_age:.2f}s), still publishing weights'
            )
        weights = self.policy.act(self.last_state, temperature=self.temperature)
        weights = np.clip(weights, 1e-3, None)
        weights = weights / np.sum(weights)

        alpha = np.clip(self.smoothing, 0.0, 1.0)
        weights = (1.0 - alpha) * self.last_weights + alpha * weights
        weights = np.clip(weights, 1e-3, None)
        weights = weights / np.sum(weights)
        self.last_weights = weights

        msg = Float64MultiArray()
        msg.data = weights.tolist()
        self.weight_pub.publish(msg)

        if now - self.last_log_time >= self.log_interval_sec:
            self.last_log_time = now
            self.get_logger().info(
                f'[RL-WEIGHT] state='
                f'[{self.last_state[0]:.3f}, {self.last_state[1]:.3f}, {self.last_state[2]:.3f}, '
                f'{self.last_state[3]:.3f}, {self.last_state[4]:.3f}, {self.last_state[5]:.3f}, {self.last_state[6]:.3f}] '
                f'weights=[{weights[0]:.3f}, {weights[1]:.3f}, {weights[2]:.3f}, {weights[3]:.3f}] '
                f'state_age={state_age:.2f}s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RLWeightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
