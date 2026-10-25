#!/usr/bin/env python3

import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cure_explore_interfaces.msg import ResetAck
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from semantic_grid_msgs.msg import SemanticGrid
from std_msgs.msg import Empty


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class OracleSemanticMapper(Node):
    def __init__(self):
        super().__init__('semantic_mapper')

        package_share = Path(get_package_share_directory('clip_semantic'))
        default_world = (
            Path(get_package_share_directory('simulation_bringup')) /
            'world' /
            'model' /
            'model_semantic.world'
        )
        default_scoring = package_share / 'config' / 'static_semantic_scores.json'
        default_static_grid = package_share / 'config' / 'static_semantic_grid_model.npz'

        self.declare_parameter('world_sdf_path', str(default_world))
        self.declare_parameter('scoring_config_path', str(default_scoring))
        self.declare_parameter('static_grid_path', str(default_static_grid))
        self.declare_parameter('frame_id', 'global_map')
        self.declare_parameter('semantic_value_scale', 1.0)
        self.declare_parameter('door_region_size', [1.6, 1.4])
        self.declare_parameter('dead_end_region_size', [1.8, 1.8])
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('semantic_build_chunk_rows', 128)

        self.world_sdf_path = Path(self.get_parameter('world_sdf_path').value)
        self.scoring_config_path = Path(self.get_parameter('scoring_config_path').value)
        self.static_grid_path = Path(self.get_parameter('static_grid_path').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.semantic_value_scale = float(self.get_parameter('semantic_value_scale').value)
        self.publish_period_sec = max(0.2, float(self.get_parameter('publish_period_sec').value))
        self.semantic_build_chunk_rows = max(
            16,
            int(self.get_parameter('semantic_build_chunk_rows').value),
        )

        self.region_sizes = {
            'door': tuple(float(v) for v in self.get_parameter('door_region_size').value),
            'dead_end': tuple(float(v) for v in self.get_parameter('dead_end_region_size').value),
        }

        self._load_scoring_config()
        self.regions = self._load_regions_from_world()

        self.global_map = None
        self.semantic_value_map = None
        self.semantic_risk_map = None
        self.semantic_info = None
        self._publish_debug_count = 0
        self._received_global_map_debug = False
        self._load_static_semantic_grid()

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            OccupancyGrid,
            '/global_map',
            self.global_map_cb,
            map_qos,
        )
        self.create_subscription(
            Empty,
            '/cure_explore/reset',
            self.global_reset_cb,
            10,
        )

        semantic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.value_pub = self.create_publisher(
            SemanticGrid, '/semantic_grid_float', semantic_qos
        )
        self.risk_pub = self.create_publisher(
            SemanticGrid, '/semantic_risk_grid_float', semantic_qos
        )
        self.reset_ack_pub = self.create_publisher(
            ResetAck,
            '/cure_explore/internal_reset_ack',
            10,
        )

        self.create_timer(self.publish_period_sec, self.publish_maps)

        counts = {}
        for region in self.regions:
            counts[region['label']] = counts.get(region['label'], 0) + 1
        self.get_logger().info(
            'Oracle semantic mapper ready: '
            f'static_grid={self.static_grid_path}, '
            f'grid={self.semantic_info.width}x{self.semantic_info.height}, '
            f'res={self.semantic_info.resolution:.3f}, '
            f'origin=({self.semantic_info.origin.position.x:.2f}, '
            f'{self.semantic_info.origin.position.y:.2f}), '
            f'world={self.world_sdf_path}, '
            f'regions={counts if counts else "none"}, '
            f'corridor_base={self.label_scores["corridor"]["semantic_value"]:.2f}, '
            f'corridor_risk={self.label_scores["corridor"]["risk_level"]:.2f}'
        )

    def _load_static_semantic_grid(self):
        if not self.static_grid_path.exists():
            raise FileNotFoundError(f'Static semantic grid not found: {self.static_grid_path}')

        data = np.load(self.static_grid_path, allow_pickle=False)
        width = int(data['width'])
        height = int(data['height'])
        resolution = float(data['resolution'])
        origin_x = float(data['origin_x'])
        origin_y = float(data['origin_y'])

        semantic = np.asarray(data['semantic'], dtype=np.float32).reshape(-1)
        risk = np.asarray(data['risk'], dtype=np.float32).reshape(-1)
        expected = width * height
        if semantic.size != expected or risk.size != expected:
            raise RuntimeError(
                'Static semantic grid size mismatch: '
                f'expected={expected}, semantic={semantic.size}, risk={risk.size}'
            )

        info = MapMetaData()
        info.resolution = resolution
        info.width = width
        info.height = height
        info.origin.position.x = origin_x
        info.origin.position.y = origin_y
        info.origin.orientation.w = 1.0

        self.semantic_info = info
        self.semantic_value_map = np.ascontiguousarray(semantic, dtype=np.float32)
        self.semantic_risk_map = np.ascontiguousarray(risk, dtype=np.float32)

        self.get_logger().info(
            'Loaded static semantic grids: '
            f'value[min={float(np.min(self.semantic_value_map)):.3f}, '
            f'max={float(np.max(self.semantic_value_map)):.3f}], '
            f'risk[min={float(np.min(self.semantic_risk_map)):.3f}, '
            f'max={float(np.max(self.semantic_risk_map)):.3f}]'
        )

    def _load_scoring_config(self):
        if not self.scoring_config_path.exists():
            raise FileNotFoundError(f'Static semantic scoring config not found: {self.scoring_config_path}')

        config = json.loads(self.scoring_config_path.read_text(encoding='utf-8'))
        self.label_scores = config['labels']
        self.label_priority = config['priority']
        self.semantic_bounds = config['semantic_bounds']
        self.risk_bounds = config['risk_bounds']
        self.corridor_semantic_influence = config['corridor_semantic_influence']
        self.corridor_risk_influence = config['corridor_risk_influence']

        self.default_semantic_value = (
            float(self.label_scores['corridor']['semantic_value']) *
            self.semantic_value_scale
        )
        self.default_semantic_risk = float(self.label_scores['corridor']['risk_level'])

    def _load_regions_from_world(self):
        if not self.world_sdf_path.exists():
            self.get_logger().error(f'Oracle world file not found: {self.world_sdf_path}')
            return []

        try:
            root = ET.parse(self.world_sdf_path).getroot()
        except Exception as exc:
            self.get_logger().error(f'Failed to parse oracle world file: {exc}')
            return []

        world = root.find('world')
        if world is None:
            self.get_logger().error('Invalid SDF: missing <world> root')
            return []

        regions = []
        for model in world.findall('model'):
            name = model.get('name', '')
            if not name.startswith('semantic_marker_'):
                continue

            label = self._label_from_model_name(name)
            if label is None:
                continue

            pose_values = self._parse_pose(model.findtext('pose', default='0 0 0 0 0 0'))
            if pose_values is None:
                continue

            size = self._extract_region_size(model, label)
            if size is None:
                continue

            regions.append({
                'name': name,
                'label': label,
                'x': pose_values[0],
                'y': pose_values[1],
                'yaw': pose_values[5],
                'sx': size[0],
                'sy': size[1],
            })

        return regions

    def _label_from_model_name(self, name):
        for prefix, label in (
            ('semantic_marker_door_', 'door'),
            ('semantic_marker_dead_end_', 'dead_end'),
            ('semantic_marker_water_', 'water'),
        ):
            if name.startswith(prefix):
                return label
        return None

    def _parse_pose(self, pose_text):
        try:
            values = [float(v) for v in pose_text.split()]
        except ValueError:
            return None
        if len(values) < 6:
            return None
        return values[:6]

    def _extract_region_size(self, model, label):
        size_text = model.findtext('.//visual/geometry/box/size')
        if size_text:
            values = [float(v) for v in size_text.split()]
            if len(values) >= 2:
                return values[0], values[1]
        return self.region_sizes.get(label)

    def global_map_cb(self, msg):
        self.global_map = msg
        if not self._received_global_map_debug:
            self._received_global_map_debug = True
            self.get_logger().info(
                'Received global_map; semantic mapper will use precomputed static grid: '
                f'frame={msg.header.frame_id}, size={msg.info.width}x{msg.info.height}, '
                f'res={msg.info.resolution:.3f}; '
                f'static_grid_size={self.semantic_info.width}x{self.semantic_info.height}, '
                f'static_origin=({self.semantic_info.origin.position.x:.2f}, '
                f'{self.semantic_info.origin.position.y:.2f})'
            )
        self.publish_maps()

    def global_reset_cb(self, msg):
        del msg
        ok = True
        ack_msg = 'static oracle semantic maps republished'
        try:
            self._load_static_semantic_grid()
        except Exception as exc:
            ok = False
            ack_msg = f'static oracle semantic reload exception: {exc}'
            self.get_logger().error(ack_msg)

        ack = ResetAck()
        ack.source = 'semantic_mapper'
        ack.success = ok
        ack.message = ack_msg
        self.reset_ack_pub.publish(ack)

        if ok:
            self.publish_maps()

    def _distance_to_region(self, wx, wy, region):
        dx = wx - region['x']
        dy = wy - region['y']
        cos_yaw = math.cos(region['yaw'])
        sin_yaw = math.sin(region['yaw'])
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        hx = region['sx'] * 0.5
        hy = region['sy'] * 0.5
        dx_out = max(abs(local_x) - hx, 0.0)
        dy_out = max(abs(local_y) - hy, 0.0)
        return math.hypot(dx_out, dy_out)

    def _nearest_distance_by_label(self, wx, wy, label):
        best = float('inf')
        for region in self.regions:
            if region['label'] != label:
                continue
            best = min(best, self._distance_to_region(wx, wy, region))
        return best

    def _influence_score(self, distance_m, cfg):
        if not math.isfinite(distance_m):
            return 0.0
        d = max(0.0, distance_m)
        mid_scale = max(1e-6, float(cfg['mid_scale_m']))
        near_scale = max(1e-6, float(cfg['near_scale_m']))
        return (
            float(cfg['mid_amp']) * math.exp(-d / mid_scale) +
            float(cfg['near_amp']) * math.exp(-d / near_scale)
        )

    def _explicit_label_at(self, wx, wy):
        best_label = None
        best_priority = -1
        for region in self.regions:
            if self._distance_to_region(wx, wy, region) > 1e-6:
                continue
            priority = int(self.label_priority.get(region['label'], 0))
            if priority > best_priority:
                best_label = region['label']
                best_priority = priority
        return best_label

    def _compute_corridor_semantic_value(self, wx, wy):
        semantic = float(self.label_scores['corridor']['semantic_value'])
        for label, cfg in self.corridor_semantic_influence.items():
            distance = self._nearest_distance_by_label(wx, wy, label)
            semantic += float(cfg['sign']) * self._influence_score(distance, cfg)
        return _clamp(
            semantic,
            float(self.semantic_bounds['min']),
            float(self.semantic_bounds['max']),
        )

    def _compute_corridor_risk_value(self, wx, wy):
        risk = float(self.label_scores['corridor']['risk_level'])
        for label, cfg in self.corridor_risk_influence.items():
            distance = self._nearest_distance_by_label(wx, wy, label)
            risk += float(cfg['sign']) * self._influence_score(distance, cfg)
        return _clamp(
            risk,
            float(self.risk_bounds['min']),
            float(self.risk_bounds['max']),
        )

    def _region_distance_grid(self, wx, wy, region):
        dx = wx[None, :] - np.float32(region['x'])
        dy = wy[:, None] - np.float32(region['y'])
        cos_yaw = np.float32(math.cos(region['yaw']))
        sin_yaw = np.float32(math.sin(region['yaw']))
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        hx = np.float32(region['sx'] * 0.5)
        hy = np.float32(region['sy'] * 0.5)
        dx_out = np.maximum(np.abs(local_x) - hx, np.float32(0.0))
        dy_out = np.maximum(np.abs(local_y) - hy, np.float32(0.0))
        return np.hypot(dx_out, dy_out).astype(np.float32, copy=False)

    def _influence_grid(self, distance, cfg):
        mid_scale = np.float32(max(1e-6, float(cfg['mid_scale_m'])))
        near_scale = np.float32(max(1e-6, float(cfg['near_scale_m'])))
        return (
            np.float32(cfg['mid_amp']) * np.exp(-distance / mid_scale) +
            np.float32(cfg['near_amp']) * np.exp(-distance / near_scale)
        ).astype(np.float32, copy=False)

    def _rebuild_semantic_grids(self):
        if self.global_map is None:
            return

        start_time = time.perf_counter()

        info = self.global_map.info
        width = int(info.width)
        height = int(info.height)
        resolution = float(info.resolution)
        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)

        semantic_map = np.empty((height, width), dtype=np.float32)
        risk_map = np.empty((height, width), dtype=np.float32)
        x_coords = (
            origin_x +
            (np.arange(width, dtype=np.float32) + np.float32(0.5)) * np.float32(resolution)
        ).astype(np.float32, copy=False)
        regions_by_label = {}
        for region in self.regions:
            regions_by_label.setdefault(region['label'], []).append(region)

        for y0 in range(0, height, self.semantic_build_chunk_rows):
            y1 = min(height, y0 + self.semantic_build_chunk_rows)
            chunk_h = y1 - y0
            y_coords = (
                origin_y +
                (np.arange(y0, y1, dtype=np.float32) + np.float32(0.5)) * np.float32(resolution)
            ).astype(np.float32, copy=False)
            semantic_chunk = np.full(
                (chunk_h, width),
                np.float32(self.default_semantic_value),
                dtype=np.float32,
            )
            risk_chunk = np.full(
                (chunk_h, width),
                np.float32(self.default_semantic_risk),
                dtype=np.float32,
            )

            for label, cfg in self.corridor_semantic_influence.items():
                label_regions = regions_by_label.get(label, [])
                if not label_regions:
                    continue
                min_dist = np.full((chunk_h, width), np.inf, dtype=np.float32)
                for region in label_regions:
                    np.minimum(
                        min_dist,
                        self._region_distance_grid(x_coords, y_coords, region),
                        out=min_dist,
                    )
                semantic_chunk += (
                    np.float32(cfg['sign']) *
                    self._influence_grid(min_dist, cfg)
                )

            for label, cfg in self.corridor_risk_influence.items():
                label_regions = regions_by_label.get(label, [])
                if not label_regions:
                    continue
                min_dist = np.full((chunk_h, width), np.inf, dtype=np.float32)
                for region in label_regions:
                    np.minimum(
                        min_dist,
                        self._region_distance_grid(x_coords, y_coords, region),
                        out=min_dist,
                    )
                risk_chunk += (
                    np.float32(cfg['sign']) *
                    self._influence_grid(min_dist, cfg)
                )

            np.clip(
                semantic_chunk,
                np.float32(self.semantic_bounds['min']),
                np.float32(self.semantic_bounds['max']),
                out=semantic_chunk,
            )
            semantic_chunk *= np.float32(self.semantic_value_scale)
            np.clip(
                risk_chunk,
                np.float32(self.risk_bounds['min']),
                np.float32(self.risk_bounds['max']),
                out=risk_chunk,
            )

            priority_chunk = np.full((chunk_h, width), -1, dtype=np.int16)
            for region in self.regions:
                priority = int(self.label_priority.get(region['label'], 0))
                if priority < 0:
                    continue
                inside = self._region_distance_grid(x_coords, y_coords, region) <= np.float32(1e-6)
                update_mask = inside & (priority > priority_chunk)
                if not np.any(update_mask):
                    continue
                label_score = self.label_scores[region['label']]
                semantic_chunk[update_mask] = (
                    np.float32(label_score['semantic_value']) *
                    np.float32(self.semantic_value_scale)
                )
                risk_chunk[update_mask] = np.float32(label_score['risk_level'])
                priority_chunk[update_mask] = priority

            semantic_map[y0:y1, :] = semantic_chunk
            risk_map[y0:y1, :] = risk_chunk

        self.semantic_value_map = semantic_map.reshape(-1)
        self.semantic_risk_map = risk_map.reshape(-1)

        semantic_min = float(np.min(self.semantic_value_map))
        semantic_max = float(np.max(self.semantic_value_map))
        risk_min = float(np.min(self.semantic_risk_map))
        risk_max = float(np.max(self.semantic_risk_map))
        center_idx = (height // 2) * width + (width // 2)
        center_sem = float(self.semantic_value_map[center_idx])
        center_risk = float(self.semantic_risk_map[center_idx])
        self.get_logger().info(
            'Semantic grids rebuilt: '
            f'value[min={semantic_min:.3f}, max={semantic_max:.3f}, center={center_sem:.3f}], '
            f'risk[min={risk_min:.3f}, max={risk_max:.3f}, center={center_risk:.3f}], '
            f'elapsed={time.perf_counter() - start_time:.2f}s'
        )

    def publish_maps(self):
        if (
            self.semantic_value_map is None or
            self.semantic_risk_map is None or
            self.semantic_info is None
        ):
            return

        now = self.get_clock().now().to_msg()

        value_msg = SemanticGrid()
        value_msg.header.stamp = now
        value_msg.header.frame_id = self.frame_id
        value_msg.info = self.semantic_info
        value_msg.data = self.semantic_value_map.astype(np.float32).tolist()
        self.value_pub.publish(value_msg)

        risk_msg = SemanticGrid()
        risk_msg.header.stamp = now
        risk_msg.header.frame_id = self.frame_id
        risk_msg.info = self.semantic_info
        risk_msg.data = self.semantic_risk_map.astype(np.float32).tolist()
        self.risk_pub.publish(risk_msg)

        if self._publish_debug_count < 3:
            self._publish_debug_count += 1
            self.get_logger().info(
                'Published semantic grids: '
                f'frame={self.frame_id}, cells={len(value_msg.data)}, '
                f'size={self.semantic_info.width}x{self.semantic_info.height}, '
                f'value0={float(value_msg.data[0]):.3f}, '
                f'value_mid={float(value_msg.data[len(value_msg.data)//2]):.3f}, '
                f'risk0={float(risk_msg.data[0]):.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OracleSemanticMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
