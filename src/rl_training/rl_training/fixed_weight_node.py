#!/usr/bin/env python3
import argparse
import re
import time
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def _parse_weights(raw: str) -> np.ndarray:
    tokens = [tok for tok in re.split(r"[\s,;]+", str(raw).strip()) if tok]
    if len(tokens) != 4:
        raise ValueError(f"固定权重维度错误，期望4，实际{len(tokens)}，输入={raw!r}")
    arr = np.asarray([float(tok) for tok in tokens], dtype=np.float32)
    arr = np.clip(arr, 1e-3, None)
    total = float(np.sum(arr))
    if total <= 1e-9:
        return np.asarray([0.25, 0.25, 0.25, 0.25], dtype=np.float32)
    return arr / total


class RLFixedWeightNode(Node):
    def __init__(self, topic: str, weights: np.ndarray, rate_hz: float, log_interval_sec: float):
        super().__init__("rl_fixed_weight_node")
        self.weight_pub = self.create_publisher(Float64MultiArray, topic, 10)
        self.weights = np.asarray(weights, dtype=np.float32).reshape(4)
        self.log_interval_sec = max(float(log_interval_sec), 0.1)
        self.last_log_time = 0.0
        self.timer = self.create_timer(1.0 / max(float(rate_hz), 0.1), self._step)
        self.get_logger().info(
            "RLFixedWeightNode started: topic=%s rate=%.2fHz log_interval=%.1fs weights=[%.3f, %.3f, %.3f, %.3f]"
            % (
                topic,
                float(rate_hz),
                self.log_interval_sec,
                float(self.weights[0]),
                float(self.weights[1]),
                float(self.weights[2]),
                float(self.weights[3]),
            )
        )

    def _step(self) -> None:
        msg = Float64MultiArray()
        msg.data = self.weights.tolist()
        self.weight_pub.publish(msg)

        now = time.time()
        if now - self.last_log_time >= self.log_interval_sec:
            self.last_log_time = now
            self.get_logger().info(
                "[RL-FIXED] weights=[%.3f, %.3f, %.3f, %.3f]"
                % (
                    float(self.weights[0]),
                    float(self.weights[1]),
                    float(self.weights[2]),
                    float(self.weights[3]),
                )
            )


def parse_args(argv: List[str]):
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", type=str, default="0.25,0.25,0.25,0.25")
    parser.add_argument("--topic", type=str, default="/rl_training/weights")
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--log-interval", type=float, default=2.0)
    return parser.parse_args(argv)


def main(args=None):
    cli = parse_args(args)
    weights = _parse_weights(cli.weights)
    rclpy.init(args=None)
    node = RLFixedWeightNode(
        topic=str(cli.topic),
        weights=weights,
        rate_hz=float(cli.rate),
        log_interval_sec=float(cli.log_interval),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
