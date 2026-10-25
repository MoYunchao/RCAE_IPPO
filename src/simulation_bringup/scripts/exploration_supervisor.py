#!/usr/bin/env python3
import csv
import json
import math
import os
import time
from pathlib import Path

import rclpy
from frontier_msgs.msg import FrontierArrayExtended
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


class ExplorationSupervisor(Node):
    def __init__(self):
        super().__init__('exploration_supervisor')
        self.declare_parameter('robot_count', 3)
        self.declare_parameter('global_map_topic', '/global_map')
        self.declare_parameter('frontier_topic', '/frontiers_info')
        self.declare_parameter('stop_topic', '/exploration_stop')
        self.declare_parameter('coverage_target', 0.95)
        self.declare_parameter('max_mission_time_sec', 600.0)
        self.declare_parameter('no_frontier_hold_sec', 10.0)
        self.declare_parameter('check_period_sec', 1.0)
        self.declare_parameter('roi_min_x', -17.4)
        self.declare_parameter('roi_min_y', -17.4)
        self.declare_parameter('roi_max_x', 17.6)
        self.declare_parameter('roi_max_y', 17.6)
        self.declare_parameter('static_map_yaml', '')
        self.declare_parameter('static_map_count_mode', 'free_only')
        self.declare_parameter('coverage_total_override', 0)
        self.declare_parameter('max_odom_step_m', 1.0)
        self.declare_parameter('csv_path', os.path.join(os.getcwd(), 'data', 'metrics', 'energy_mission_metrics.csv'))
        self.declare_parameter('summary_path', os.path.join(os.getcwd(), 'data', 'metrics', 'energy_mission_summary.json'))
        self.declare_parameter('coverage_timeseries_path', '')
        self.declare_parameter('battery_timeseries_path', '')

        self.robot_count = int(self.get_parameter('robot_count').value)
        self.coverage_target = float(self.get_parameter('coverage_target').value)
        self.max_mission_time_sec = float(self.get_parameter('max_mission_time_sec').value)
        self.no_frontier_hold_sec = float(self.get_parameter('no_frontier_hold_sec').value)
        self.check_period_sec = float(self.get_parameter('check_period_sec').value)
        self.roi_min_x = float(self.get_parameter('roi_min_x').value)
        self.roi_min_y = float(self.get_parameter('roi_min_y').value)
        self.roi_max_x = float(self.get_parameter('roi_max_x').value)
        self.roi_max_y = float(self.get_parameter('roi_max_y').value)
        self.static_map_yaml = str(self.get_parameter('static_map_yaml').value).strip()
        self.static_map_count_mode = str(self.get_parameter('static_map_count_mode').value).strip().lower()
        self.coverage_total_override = int(self.get_parameter('coverage_total_override').value)
        self.max_odom_step_m = float(self.get_parameter('max_odom_step_m').value)
        self.csv_path = str(self.get_parameter('csv_path').value)
        self.summary_path = str(self.get_parameter('summary_path').value)
        self.coverage_timeseries_path = str(self.get_parameter('coverage_timeseries_path').value).strip()
        if not self.coverage_timeseries_path:
            self.coverage_timeseries_path = str(Path(self.csv_path).with_name('coverage_timeseries.csv'))
        self.battery_timeseries_path = str(self.get_parameter('battery_timeseries_path').value).strip()
        if not self.battery_timeseries_path:
            self.battery_timeseries_path = str(Path(self.csv_path).with_name('battery_timeseries.csv'))

        self.map_msg = None
        self.static_map_meta = None
        self._coverage_roi_indices = None
        self._coverage_roi_total = None
        self.start_time = time.monotonic()
        self.stopped = False
        self.frontier_count = None
        self.last_frontier_msg_time = self.start_time
        self.no_frontier_since = None
        self.t90_sec = None
        self.t95_sec = None
        self.path_lengths = [0.0] * self.robot_count
        self.last_odom_xy = [None] * self.robot_count
        self.failed = [False] * self.robot_count
        self.failure_time_sec = [None] * self.robot_count
        self.final_battery = [None] * self.robot_count
        self.current_battery = [None] * self.robot_count

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('global_map_topic').value), self.map_callback, map_qos)
        self.create_subscription(
            FrontierArrayExtended, str(self.get_parameter('frontier_topic').value), self.frontier_callback, 10)
        for rid in range(self.robot_count):
            self.create_subscription(
                Odometry, f'/robot_{rid}/odom',
                lambda msg, robot_id=rid: self.odom_callback(msg, robot_id), 20)
            self.create_subscription(
                Bool, f'/robot_{rid}/failed',
                lambda msg, robot_id=rid: self.failed_callback(msg, robot_id),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(
                BatteryState, f'/robot_{rid}/battery_state',
                lambda msg, robot_id=rid: self.battery_callback(msg, robot_id), 10)

        self.stop_pub = self.create_publisher(
            Bool, str(self.get_parameter('stop_topic').value), stop_qos)
        self.timer = self.create_timer(self.check_period_sec, self.on_timer)
        max_time_desc = 'disabled' if self.max_mission_time_sec <= 0.0 else f'{self.max_mission_time_sec:.1f}s'
        self.get_logger().info(
            f'Energy mission supervisor started: target={self.coverage_target:.2f}, '
            f'Tmax={max_time_desc}')
        self._try_load_static_map()

    def map_callback(self, msg):
        self.map_msg = msg
        if self.static_map_meta is not None and self._coverage_roi_indices is None:
            self._build_static_roi_mask()

    @staticmethod
    def _read_simple_yaml_map(path):
        data = {}
        for raw in path.read_text(encoding='utf-8').splitlines():
            line = raw.strip()
            if not line or line.startswith('#') or ':' not in line:
                continue
            key, value = line.split(':', 1)
            data[key.strip()] = value.strip()
        image = data.get('image', '').strip().strip('"').strip("'")
        resolution = float(data.get('resolution', '0.05'))
        origin_text = data.get('origin', '[0.0, 0.0, 0.0]').strip()
        negate = int(float(data.get('negate', '0')))
        occupied_thresh = float(data.get('occupied_thresh', '0.65'))
        free_thresh = float(data.get('free_thresh', '0.196'))
        if not image:
            raise RuntimeError(f'Invalid map yaml, missing image: {path}')
        if not (origin_text.startswith('[') and origin_text.endswith(']')):
            raise RuntimeError(f'Invalid origin in map yaml: {path}')
        origin_vals = [float(x.strip()) for x in origin_text[1:-1].split(',')]
        if len(origin_vals) < 2:
            raise RuntimeError(f'Invalid origin in map yaml: {path}')
        return {
            'image_path': (path.parent / image).resolve(),
            'resolution': resolution,
            'origin_x': origin_vals[0],
            'origin_y': origin_vals[1],
            'negate': negate,
            'occupied_thresh': occupied_thresh,
            'free_thresh': free_thresh,
        }

    @staticmethod
    def _read_pgm(path):
        with open(path, 'rb') as handle:
            magic = handle.readline().strip()
            if magic not in (b'P5', b'P2'):
                raise RuntimeError(f'Unsupported PGM format: {magic!r} ({path})')

            def tokens():
                while True:
                    line = handle.readline()
                    if not line:
                        return
                    line = line.strip()
                    if not line or line.startswith(b'#'):
                        continue
                    for token in line.split():
                        yield token

            token_iter = tokens()
            width = int(next(token_iter))
            height = int(next(token_iter))
            maxval = int(next(token_iter))
            if magic == b'P5':
                raw = handle.read(width * height)
                if len(raw) != width * height:
                    raise RuntimeError(f'PGM size mismatch: {path}')
                pixels = list(raw)
            else:
                rest = list(token_iter)
                if len(rest) != width * height:
                    raise RuntimeError(f'PGM size mismatch: {path}')
                pixels = [int(token) for token in rest]
        return width, height, maxval, pixels

    def _try_load_static_map(self):
        if not self.static_map_yaml:
            return
        yaml_path = Path(self.static_map_yaml).expanduser().resolve()
        if not yaml_path.exists():
            self.get_logger().warn(f'static_map_yaml not found: {yaml_path}')
            return
        try:
            meta = self._read_simple_yaml_map(yaml_path)
            width, height, maxval, pixels = self._read_pgm(meta['image_path'])
        except Exception as exc:
            self.get_logger().warn(f'Failed to load static map yaml/image: {exc}')
            return
        meta.update({'width': width, 'height': height, 'maxval': maxval, 'pixels': pixels})
        self.static_map_meta = meta
        self.get_logger().info(
            f'Loaded static map for fixed coverage denominator: yaml={yaml_path} '
            f'image={meta["image_path"]} size={width}x{height} '
            f'res={meta["resolution"]:.3f} mode={self.static_map_count_mode}')

    def _static_cell_state(self, sx, sy):
        meta = self.static_map_meta
        if meta is None or sx < 0 or sx >= meta['width'] or sy < 0 or sy >= meta['height']:
            return None
        pixel = meta['pixels'][sy * meta['width'] + sx]
        occ = float(pixel) / float(meta['maxval'])
        if meta['negate'] == 0:
            occ = 1.0 - occ
        if occ > meta['occupied_thresh']:
            return 'occupied'
        if occ < meta['free_thresh']:
            return 'free'
        return 'unknown'

    @staticmethod
    def world_to_grid(map_msg, wx, wy):
        res = float(map_msg.info.resolution)
        ox = float(map_msg.info.origin.position.x)
        oy = float(map_msg.info.origin.position.y)
        return int(math.floor((wx - ox) / res)), int(math.floor((wy - oy) / res))

    def _build_static_roi_mask(self):
        if self.map_msg is None or self.static_map_meta is None:
            return
        msg = self.map_msg
        width, height = int(msg.info.width), int(msg.info.height)
        gx0, gy0 = self.world_to_grid(msg, self.roi_min_x, self.roi_min_y)
        gx1, gy1 = self.world_to_grid(msg, self.roi_max_x, self.roi_max_y)
        min_gx = max(0, min(gx0, gx1))
        max_gx = min(width - 1, max(gx0, gx1))
        min_gy = max(0, min(gy0, gy1))
        max_gy = min(height - 1, max(gy0, gy1))
        indices = []
        for gy in range(min_gy, max_gy + 1):
            row_offset = gy * width
            wy = msg.info.origin.position.y + (gy + 0.5) * msg.info.resolution
            for gx in range(min_gx, max_gx + 1):
                wx = msg.info.origin.position.x + (gx + 0.5) * msg.info.resolution
                sx = int(math.floor((wx - self.static_map_meta['origin_x']) / self.static_map_meta['resolution']))
                sy = int(math.floor((wy - self.static_map_meta['origin_y']) / self.static_map_meta['resolution']))
                state = self._static_cell_state(sx, sy)
                if self.static_map_count_mode == 'free_only':
                    keep = state == 'free'
                elif self.static_map_count_mode == 'known_non_occupied':
                    keep = state in ('free', 'unknown')
                else:
                    keep = state is not None and state != 'occupied'
                if keep:
                    indices.append(row_offset + gx)
        self._coverage_roi_indices = indices
        self._coverage_roi_total = len(indices)
        self.get_logger().info(
            f'Built static ROI coverage mask: total_explorable_cells={self._coverage_roi_total} '
            f'within ROI=({self.roi_min_x:.2f},{self.roi_min_y:.2f})-({self.roi_max_x:.2f},{self.roi_max_y:.2f})')

    def frontier_callback(self, msg):
        self.frontier_count = len(msg.frontiers_extended)
        self.last_frontier_msg_time = time.monotonic()
        if self.frontier_count > 0:
            self.no_frontier_since = None

    def battery_callback(self, msg, robot_id):
        percentage = float(msg.percentage)
        self.current_battery[robot_id] = percentage
        self.final_battery[robot_id] = percentage
        if not msg.present:
            self.mark_failed(robot_id)

    def failed_callback(self, msg, robot_id):
        if msg.data:
            self.mark_failed(robot_id)

    def mark_failed(self, robot_id):
        if self.failed[robot_id]:
            return
        self.failed[robot_id] = True
        self.failure_time_sec[robot_id] = time.monotonic() - self.start_time
        self.get_logger().warn(
            f'robot_{robot_id} failed at {self.failure_time_sec[robot_id]:.1f}s; '
            f'ATL frozen at {self.path_lengths[robot_id]:.2f}m')

    def odom_callback(self, msg, robot_id):
        xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        prev = self.last_odom_xy[robot_id]
        self.last_odom_xy[robot_id] = xy
        if prev is None or self.failed[robot_id]:
            return
        dist = math.hypot(xy[0] - prev[0], xy[1] - prev[1])
        if 1e-5 < dist <= self.max_odom_step_m:
            self.path_lengths[robot_id] += dist

    def compute_roi_coverage(self):
        if self.map_msg is None:
            return None, 0, 0
        msg = self.map_msg

        if self.static_map_meta is not None:
            if self._coverage_roi_indices is None:
                self._build_static_roi_mask()
            indices = self._coverage_roi_indices or []
            total = int(self._coverage_roi_total or 0)
            if total <= 0:
                return None, 0, 0
            known = 0
            for idx in indices:
                if 0 <= idx < len(msg.data) and msg.data[idx] != -1:
                    known += 1
            if self.coverage_total_override > 0:
                total = self.coverage_total_override
            return min(1.0, known / max(1, total)), known, total

        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        width, height = int(msg.info.width), int(msg.info.height)
        x0 = max(0, int(math.floor((self.roi_min_x - ox) / res)))
        x1 = min(width - 1, int(math.floor((self.roi_max_x - ox) / res)))
        y0 = max(0, int(math.floor((self.roi_min_y - oy) / res)))
        y1 = min(height - 1, int(math.floor((self.roi_max_y - oy) / res)))
        if x0 > x1 or y0 > y1:
            return None, 0, 0
        known = 0
        total = 0
        for y in range(y0, y1 + 1):
            offset = y * width
            for x in range(x0, x1 + 1):
                total += 1
                known += int(msg.data[offset + x] != -1)
        if self.coverage_total_override > 0:
            total = self.coverage_total_override
        return min(1.0, known / max(1, total)), known, total

    def write_timeseries(self, runtime_sec, coverage, known, total, stop_reason=''):
        path = Path(self.coverage_timeseries_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fields = ['runtime_sec', 'coverage_ratio', 'known_cells', 'total_roi_cells',
                  'ATL_m', 'NFR', 'frontier_count', 'stop_reason']
        row = {
            'runtime_sec': f'{runtime_sec:.3f}',
            'coverage_ratio': f'{coverage:.6f}',
            'known_cells': known,
            'total_roi_cells': total,
            'ATL_m': f'{sum(self.path_lengths):.3f}',
            'NFR': sum(self.failed),
            'frontier_count': self.frontier_count if self.frontier_count is not None else '',
            'stop_reason': stop_reason,
        }
        new_file = not path.exists()
        with path.open('a', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            if new_file:
                writer.writeheader()
            writer.writerow(row)

    def write_battery_timeseries(self, runtime_sec, coverage, stop_reason=''):
        path = Path(self.battery_timeseries_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        robot_fields = [f'robot_{rid}_battery' for rid in range(self.robot_count)]
        fields = ['runtime_sec', 'coverage_ratio', 'mean_battery', 'min_battery', 'max_battery',
                  'NFR', 'stop_reason'] + robot_fields
        known_batteries = [battery for battery in self.current_battery if battery is not None]
        row = {
            'runtime_sec': f'{runtime_sec:.3f}',
            'coverage_ratio': f'{coverage:.6f}',
            'mean_battery': '' if not known_batteries else f'{sum(known_batteries) / len(known_batteries):.6f}',
            'min_battery': '' if not known_batteries else f'{min(known_batteries):.6f}',
            'max_battery': '' if not known_batteries else f'{max(known_batteries):.6f}',
            'NFR': sum(self.failed),
            'stop_reason': stop_reason,
        }
        for rid in range(self.robot_count):
            battery = self.current_battery[rid]
            row[f'robot_{rid}_battery'] = '' if battery is None else f'{battery:.6f}'
        new_file = not path.exists()
        with path.open('a', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            if new_file:
                writer.writeheader()
            writer.writerow(row)

    def finish(self, reason, runtime_sec, coverage, known, total):
        if self.stopped:
            return
        self.stopped = True
        payload = {
            'type': 'energy_constrained_mission_summary',
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'stop_reason': reason,
            'runtime_sec': round(runtime_sec, 3),
            'FCR': round(coverage, 6),
            'T90_sec': None if self.t90_sec is None else round(self.t90_sec, 3),
            'T95_sec': None if self.t95_sec is None else round(self.t95_sec, 3),
            'T90_display': '>Tmax' if self.t90_sec is None else f'{self.t90_sec:.3f}',
            'T95_display': '>Tmax' if self.t95_sec is None else f'{self.t95_sec:.3f}',
            'ATL_m': round(sum(self.path_lengths), 3),
            'NFR': int(sum(self.failed)),
            'known_cells': int(known),
            'total_roi_cells': int(total),
            'robot_metrics': [
                {
                    'robot_id': rid,
                    'failed': self.failed[rid],
                    'failure_time_sec': self.failure_time_sec[rid],
                    'active_trajectory_length_m': round(self.path_lengths[rid], 3),
                    'final_battery': self.final_battery[rid],
                }
                for rid in range(self.robot_count)
            ],
        }
        summary_path = Path(self.summary_path)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
        csv_path = Path(self.csv_path)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        new_file = not csv_path.exists()
        with csv_path.open('a', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(payload.keys())[:-1])
            if new_file:
                writer.writeheader()
            writer.writerow({key: value for key, value in payload.items() if key != 'robot_metrics'})
        self.write_timeseries(runtime_sec, coverage, known, total, reason)
        self.write_battery_timeseries(runtime_sec, coverage, reason)
        stop = Bool()
        stop.data = True
        self.stop_pub.publish(stop)
        self.get_logger().warn(
            f'Mission stopped: reason={reason} FCR={coverage:.3f} ATL={sum(self.path_lengths):.1f}m '
            f'NFR={sum(self.failed)}')

    def on_timer(self):
        if self.stopped:
            return
        coverage, known, total = self.compute_roi_coverage()
        if coverage is None:
            return
        now = time.monotonic()
        runtime_sec = now - self.start_time
        if self.t90_sec is None and coverage >= 0.90:
            self.t90_sec = runtime_sec
        if self.t95_sec is None and coverage >= 0.95:
            self.t95_sec = runtime_sec
        self.write_timeseries(runtime_sec, coverage, known, total)
        self.write_battery_timeseries(runtime_sec, coverage)

        reason = None
        if coverage >= self.coverage_target:
            reason = 'coverage_target_reached'
        elif self.max_mission_time_sec > 0.0 and runtime_sec >= self.max_mission_time_sec:
            reason = 'max_mission_time'
        elif all(self.failed):
            reason = 'all_robots_failed'
        elif self.frontier_count == 0 or now - self.last_frontier_msg_time >= self.no_frontier_hold_sec:
            if self.no_frontier_since is None:
                self.no_frontier_since = now
            elif now - self.no_frontier_since >= self.no_frontier_hold_sec:
                reason = 'no_reachable_frontier'
        if reason:
            self.finish(reason, runtime_sec, coverage, known, total)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
