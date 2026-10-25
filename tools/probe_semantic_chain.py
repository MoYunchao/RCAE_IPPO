#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from frontier_msgs.msg import FrontierArrayExtended
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.topic_endpoint_info import TopicEndpointInfo
from semantic_grid_msgs.msg import SemanticGrid


def _qos_name(value: object) -> str:
    name = getattr(value, 'name', None)
    if name:
        return str(name).lower()
    text = str(value)
    return text.split('.')[-1].lower()


def endpoint_line(info: TopicEndpointInfo) -> str:
    qos = info.qos_profile
    return (
        f'{info.node_namespace.rstrip("/")}/{info.node_name} '
        f'type={info.topic_type} '
        f'qos(rel={_qos_name(qos.reliability)}, '
        f'dur={_qos_name(qos.durability)}, depth={qos.depth})'
    )


def grid_stats(msg: SemanticGrid) -> str:
    data = np.asarray(msg.data, dtype=np.float32)
    if data.size == 0:
        return 'empty'
    finite = data[np.isfinite(data)]
    if finite.size == 0:
        return f'cells={data.size}, finite=0'
    nonzero = int(np.count_nonzero(np.abs(finite) > 1e-6))
    return (
        f'frame={msg.header.frame_id}, cells={data.size}, '
        f'size={msg.info.width}x{msg.info.height}, res={msg.info.resolution:.3f}, '
        f'min={float(np.min(finite)):.3f}, max={float(np.max(finite)):.3f}, '
        f'mean={float(np.mean(finite)):.3f}, nonzero={nonzero}'
    )


def sample_grid(msg: Optional[SemanticGrid], x: float, y: float) -> Tuple[Optional[float], str]:
    if msg is None:
        return None, 'no_grid'
    width = int(msg.info.width)
    height = int(msg.info.height)
    res = float(msg.info.resolution)
    if width <= 0 or height <= 0 or res <= 0.0:
        return None, 'bad_info'
    gx = int(math.floor((float(x) - float(msg.info.origin.position.x)) / res))
    gy = int(math.floor((float(y) - float(msg.info.origin.position.y)) / res))
    if gx < 0 or gy < 0 or gx >= width or gy >= height:
        return None, f'out_of_bounds(gx={gx}, gy={gy})'
    idx = gy * width + gx
    if idx < 0 or idx >= len(msg.data):
        return None, f'bad_idx(idx={idx})'
    return float(msg.data[idx]), f'idx={idx}, gx={gx}, gy={gy}'


class SemanticChainProbe(Node):
    def __init__(self) -> None:
        super().__init__('semantic_chain_probe')
        self.semantic_grid: Optional[SemanticGrid] = None
        self.risk_grid: Optional[SemanticGrid] = None
        self.frontiers: Optional[FrontierArrayExtended] = None
        self.semantic_source = ''
        self.risk_source = ''

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            SemanticGrid,
            '/semantic_grid_float',
            lambda msg: self._semantic_cb(msg, 'transient_local'),
            transient_qos,
        )
        self.create_subscription(
            SemanticGrid,
            '/semantic_grid_float',
            lambda msg: self._semantic_cb(msg, 'volatile'),
            volatile_qos,
        )
        self.create_subscription(
            SemanticGrid,
            '/semantic_risk_grid_float',
            lambda msg: self._risk_cb(msg, 'transient_local'),
            transient_qos,
        )
        self.create_subscription(
            SemanticGrid,
            '/semantic_risk_grid_float',
            lambda msg: self._risk_cb(msg, 'volatile'),
            volatile_qos,
        )
        self.create_subscription(
            FrontierArrayExtended,
            '/frontiers_info',
            self._frontier_cb,
            10,
        )

    def _semantic_cb(self, msg: SemanticGrid, source: str) -> None:
        self.semantic_grid = msg
        self.semantic_source = source

    def _risk_cb(self, msg: SemanticGrid, source: str) -> None:
        self.risk_grid = msg
        self.risk_source = source

    def _frontier_cb(self, msg: FrontierArrayExtended) -> None:
        self.frontiers = msg

    def print_endpoint_report(self) -> None:
        print('=== ROS graph endpoint report ===')
        for topic in (
            '/global_map',
            '/semantic_grid_float',
            '/semantic_risk_grid_float',
            '/frontiers_info',
        ):
            pubs = self.get_publishers_info_by_topic(topic)
            subs = self.get_subscriptions_info_by_topic(topic)
            print(f'{topic}: publishers={len(pubs)}, subscribers={len(subs)}')
            for info in pubs:
                print(f'  pub: {endpoint_line(info)}')
            for info in subs:
                print(f'  sub: {endpoint_line(info)}')


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Probe semantic_grid_float -> frontiers_info without relying on node logs.'
    )
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--max-frontiers', type=int, default=12)
    args = parser.parse_args()

    rclpy.init()
    node = SemanticChainProbe()
    time.sleep(0.3)
    node.print_endpoint_report()

    deadline = time.time() + max(0.5, float(args.timeout))
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.semantic_grid is not None and node.risk_grid is not None and node.frontiers is not None:
            break

    print('=== Topic receive status ===')
    print(
        f'/semantic_grid_float: {"OK" if node.semantic_grid is not None else "MISSING"}'
        f'{f" ({node.semantic_source})" if node.semantic_source else ""}'
    )
    print(
        f'/semantic_risk_grid_float: {"OK" if node.risk_grid is not None else "MISSING"}'
        f'{f" ({node.risk_source})" if node.risk_source else ""}'
    )
    print(f'/frontiers_info: {"OK" if node.frontiers is not None else "MISSING"}')

    print('\n=== Grid statistics ===')
    print(f'semantic: {grid_stats(node.semantic_grid) if node.semantic_grid is not None else "missing"}')
    print(f'risk:     {grid_stats(node.risk_grid) if node.risk_grid is not None else "missing"}')

    if node.frontiers is None:
        node.destroy_node()
        rclpy.shutdown()
        return 2

    frontiers = list(node.frontiers.frontiers_extended)
    print(f'\n=== Frontier center samples, count={len(frontiers)} ===')
    for frontier in frontiers[: max(0, int(args.max_frontiers))]:
        x = float(frontier.position[0])
        y = float(frontier.position[1])
        semantic_value, semantic_where = sample_grid(node.semantic_grid, x, y)
        risk_value, risk_where = sample_grid(node.risk_grid, x, y)
        semantic_txt = 'None' if semantic_value is None else f'{semantic_value:.3f}'
        risk_txt = 'None' if risk_value is None else f'{risk_value:.3f}'
        print(
            f'id={int(frontier.id):3d} center=({x:7.2f},{y:7.2f}) '
            f'frontier_semantic={float(frontier.semantic_score):7.3f} '
            f'grid_semantic={semantic_txt:>7s} [{semantic_where}] '
            f'risk_level={float(frontier.risk_level):7.3f} '
            f'grid_risk={risk_txt:>7s} [{risk_where}]'
        )

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
