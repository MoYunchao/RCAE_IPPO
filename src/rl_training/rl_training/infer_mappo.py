#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from frontier_msgs.msg import FrontierArrayExtended
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

try:
    import torch
except Exception:
    torch = None

from .mappo_policy import MAPPOPolicy


def _parse_bool(raw: str, default: bool = True) -> bool:
    if raw is None:
        return bool(default)
    txt = str(raw).strip().lower()
    if txt in ('1', 'true', 'yes', 'y', 'on'):
        return True
    if txt in ('0', 'false', 'no', 'n', 'off'):
        return False
    return bool(default)


def _atomic_write_json(path: Path, data: Dict[str, Any]) -> None:
    tmp = path.with_suffix(path.suffix + '.tmp')
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=True, indent=2)
    tmp.replace(path)


def _semantic_norm(value: float) -> float:
    return float(np.clip((float(value) + 0.5) / 1.5, 0.0, 1.0))


def _build_local_obs_legacy(global_state: np.ndarray, per_robot_state: np.ndarray, robot_count: int) -> np.ndarray:
    rows: List[np.ndarray] = []
    for rid in range(robot_count):
        base = rid * 4
        battery = float(per_robot_state[base + 0])
        cpu = float(per_robot_state[base + 1])
        bandwidth = float(per_robot_state[base + 2])
        rows.append(
            np.asarray(
                [
                    np.clip(battery, 0.0, 1.0),
                    np.clip(cpu, 0.0, 1.0),
                    np.clip(bandwidth, 0.0, 1.0),
                    np.clip(global_state[3], 0.0, 1.0),
                    np.clip(global_state[4], 0.0, 1.0),
                    np.clip(global_state[5], 0.0, 1.0),
                    np.clip(global_state[6], 0.0, 1.0),
                ],
                dtype=np.float32,
            )
        )
    return np.stack(rows, axis=0).astype(np.float32)


def _load_home_positions(home_pose_path: str) -> Dict[str, Tuple[float, float]]:
    path = Path(home_pose_path).expanduser().resolve()
    if not path.is_file():
        return {}

    current_robot: Optional[str] = None
    parsed: Dict[str, Dict[str, float]] = {}
    robot_re = re.compile(r'^(robot_[^:]+):\s*$')
    value_re = re.compile(r'^(x|y):\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*$')

    with open(path, 'r', encoding='utf-8') as f:
        for raw in f:
            line = raw.split('#', 1)[0].rstrip()
            stripped = line.strip()
            if not stripped:
                continue
            if stripped == 'robot_poses:':
                current_robot = None
                continue
            m_robot = robot_re.match(stripped)
            if m_robot is not None:
                current_robot = str(m_robot.group(1))
                parsed.setdefault(current_robot, {})
                continue
            if current_robot is None:
                continue
            m_value = value_re.match(stripped)
            if m_value is None:
                continue
            key = str(m_value.group(1))
            parsed[current_robot][key] = float(m_value.group(2))

    out: Dict[str, Tuple[float, float]] = {}
    for robot_name, values in parsed.items():
        if 'x' in values and 'y' in values:
            out[robot_name] = (float(values['x']), float(values['y']))
    return out


def _infer_action_mode(payload: Dict[str, Any], requested_mode: str) -> str:
    mode = str(requested_mode or 'auto').strip().lower()
    if mode in ('beta', 'dirichlet'):
        return mode

    stored_mode = str(payload.get('action_mode', '')).strip().lower()
    if stored_mode in ('beta', 'dirichlet'):
        return stored_mode

    state_dict = payload.get('policy_state_dict', {})
    if isinstance(state_dict, dict):
        head = state_dict.get('actor.net.4.weight')
        if head is not None and hasattr(head, 'shape') and len(head.shape) >= 1:
            out_dim = int(head.shape[0])
            if out_dim == 4:
                return 'dirichlet'
            if out_dim == 8:
                return 'beta'

    return 'beta'


class MAPPOInferenceLogger:
    def __init__(
        self,
        model_path: str,
        rate_hz: float,
        log_interval_sec: float,
        save_enabled: bool,
        save_root: str,
        run_name: str,
        robot_count: int,
        robot_namespace: str,
    ) -> None:
        self.enabled = bool(save_enabled)
        self.finalized = False
        self.model_path = str(Path(model_path).resolve())
        self.start_wall = time.time()
        self.step_count = 0
        self.coverage_first: Optional[float] = None
        self.coverage_last: Optional[float] = None
        self.coverage_max: Optional[float] = None
        self.inference_ms_sum = 0.0
        self.inference_ms_max = 0.0
        self.run_dir: Optional[Path] = None
        self.logs_dir: Optional[Path] = None
        self.step_metrics_path: Optional[Path] = None
        self.task_metrics_path: Optional[Path] = None
        self.summary_path: Optional[Path] = None
        self.config_path: Optional[Path] = None
        self._step_fp = None
        self._task_fp = None

        if not self.enabled:
            return

        self.run_dir = self._prepare_run_dir(save_root=save_root, run_name=run_name)
        self.logs_dir = self.run_dir / 'logs'
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.step_metrics_path = self.logs_dir / 'step_metrics.jsonl'
        self.task_metrics_path = self.logs_dir / 'task_metrics.jsonl'
        self.summary_path = self.run_dir / 'summary.json'
        self.config_path = self.run_dir / 'config.json'
        # Line buffering keeps step_metrics.jsonl visible while the node is still running.
        self._step_fp = open(self.step_metrics_path, 'a', encoding='utf-8', buffering=1)
        self._task_fp = open(self.task_metrics_path, 'a', encoding='utf-8', buffering=1)

        _atomic_write_json(
            self.config_path,
            {
                'type': 'mappo_inference_run',
                'started_at_unix': float(self.start_wall),
                'started_at_local': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.start_wall)),
                'model_path': self.model_path,
                'rate_hz': float(rate_hz),
                'log_interval_sec': float(log_interval_sec),
                'robot_count': int(robot_count),
                'robot_namespace': str(robot_namespace),
                'run_dir': str(self.run_dir),
            },
        )

    @staticmethod
    def _prepare_run_dir(save_root: str, run_name: str) -> Path:
        root = Path(save_root).expanduser()
        if not root.is_absolute():
            root = Path.cwd() / root
        root.mkdir(parents=True, exist_ok=True)

        base_name = run_name.strip() if run_name and run_name.strip() else time.strftime(
            'mappo_infer_%Y%m%d_%H%M%S'
        )
        candidate = root / base_name
        if not candidate.exists():
            return candidate

        for idx in range(1, 10000):
            trial = root / f'{base_name}_{idx:03d}'
            if not trial.exists():
                return trial
        raise RuntimeError(f'Unable to allocate run directory under {root}')

    def run_dir_str(self) -> str:
        if self.run_dir is None:
            return ''
        return str(self.run_dir)

    def record_step(
        self,
        *,
        global_state: np.ndarray,
        per_robot_state: np.ndarray,
        local_obs: np.ndarray,
        weights: np.ndarray,
        inference_ms: float,
    ) -> None:
        if not self.enabled or self.finalized or self._step_fp is None:
            return

        now = time.time()
        cov = float(np.clip(global_state[3], 0.0, 1.0))
        self.step_count += 1
        self.inference_ms_sum += float(inference_ms)
        self.inference_ms_max = max(self.inference_ms_max, float(inference_ms))

        if self.coverage_first is None:
            self.coverage_first = cov
        self.coverage_last = cov
        if self.coverage_max is None:
            self.coverage_max = cov
        else:
            self.coverage_max = max(self.coverage_max, cov)

        row = {
            'type': 'mappo_inference_step',
            'step': int(self.step_count),
            'timestamp_unix': float(now),
            'elapsed_sec': float(now - self.start_wall),
            'policy_inference_ms': float(inference_ms),
            'global_state': [float(x) for x in global_state[:7].tolist()],
            'per_robot_state': [float(x) for x in per_robot_state.tolist()],
            'local_obs_by_robot': [
                [float(x) for x in local_obs[rid].tolist()]
                for rid in range(local_obs.shape[0])
            ],
            'weights_by_robot': [
                [float(x) for x in weights[rid].tolist()]
                for rid in range(weights.shape[0])
            ],
            'coverage_ratio': cov,
            'online_norm': float(global_state[4]),
            'frontier_norm': float(global_state[5]),
            'conflict_norm': float(global_state[6]),
        }
        self._step_fp.write(json.dumps(row, ensure_ascii=True) + '\n')
        self._step_fp.flush()

    def record_task_event(self, *, robot_name: str, raw_payload: str) -> None:
        if not self.enabled or self.finalized or self._task_fp is None:
            return

        now = time.time()
        text = str(raw_payload or '').strip()
        row: Dict[str, Any] = {
            'type': 'navigator_task_event',
            'source_robot': str(robot_name),
            'logger_recorded_at_unix': float(now),
            'logger_elapsed_sec': float(now - self.start_wall),
        }

        if text:
            try:
                payload = json.loads(text)
            except Exception:
                payload = None
                row['raw_payload'] = text
            if isinstance(payload, dict):
                row['event'] = str(payload.get('event', ''))
                for key in ('phase', 'frontier_id', 'task_counter', 'score', 'goal_type'):
                    if key in payload:
                        row[key] = payload[key]
                row['payload'] = payload

        self._task_fp.write(json.dumps(row, ensure_ascii=True) + '\n')
        self._task_fp.flush()

    def finalize(self, done_reason: str, last_global_state: np.ndarray) -> None:
        if self.finalized:
            return
        self.finalized = True

        if not self.enabled:
            return

        now = time.time()
        steps = max(self.step_count, 0)
        mean_inference_ms = self.inference_ms_sum / float(steps) if steps > 0 else 0.0
        final_cov = float(self.coverage_last) if self.coverage_last is not None else float(
            np.clip(last_global_state[3], 0.0, 1.0)
        )
        max_cov = float(self.coverage_max) if self.coverage_max is not None else final_cov

        summary = {
            'type': 'mappo_inference_summary',
            'run_dir': str(self.run_dir) if self.run_dir is not None else '',
            'model_path': self.model_path,
            'started_at_unix': float(self.start_wall),
            'ended_at_unix': float(now),
            'started_at_local': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.start_wall)),
            'ended_at_local': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now)),
            'done_reason': str(done_reason or 'node_shutdown'),
            'steps_logged': int(steps),
            'time_metrics': {
                'total_time_sec': float(now - self.start_wall),
            },
            'compare_metrics': {
                'coverage_start': float(self.coverage_first) if self.coverage_first is not None else final_cov,
                'coverage_final': final_cov,
                'coverage_max': max_cov,
                'mean_policy_inference_ms': float(mean_inference_ms),
                'max_policy_inference_ms': float(self.inference_ms_max),
                'frontier_norm_final': float(np.clip(last_global_state[5], 0.0, 1.0)),
                'conflict_norm_final': float(np.clip(last_global_state[6], 0.0, 1.0)),
            },
            'files': {
                'step_metrics': str(self.step_metrics_path) if self.step_metrics_path is not None else '',
                'task_metrics': str(self.task_metrics_path) if self.task_metrics_path is not None else '',
            },
        }
        if self.summary_path is not None:
            _atomic_write_json(self.summary_path, summary)

        if self._step_fp is not None:
            self._step_fp.close()
            self._step_fp = None
        if self._task_fp is not None:
            self._task_fp.close()
            self._task_fp = None


class MAPPOInferenceNode(Node):
    def __init__(
        self,
        *,
        model_path: str,
        robot_count: int,
        robot_namespace: str,
        global_state_topic: str,
        per_robot_state_topic: str,
        frontier_topic: str,
        odom_topic_template: str,
        weight_topic_template: str,
        task_metrics_topic_template: str,
        rate_hz: float,
        log_interval_sec: float,
        save_enabled: bool,
        save_root: str,
        run_name: str,
        opportunity_top_k: int,
        candidate_distance_max_m: float,
        overlap_distance_threshold_m: float,
        battery_per_meter: float,
        battery_return_margin: float,
        battery_return_reserve: float,
        low_battery_threshold: float,
        home_pose_path: str,
        action_mode: str,
    ) -> None:
        super().__init__('rl_infer_mappo')

        if torch is None:
            raise RuntimeError(
                "PyTorch is not available. Activate the conda env that contains torch before launching MAPPO inference."
            )

        self.robot_count = int(robot_count)
        self.robot_namespace = str(robot_namespace)
        self.global_state_topic = str(global_state_topic)
        self.per_robot_state_topic = str(per_robot_state_topic)
        self.frontier_topic = str(frontier_topic)
        self.odom_topic_template = str(odom_topic_template)
        self.weight_topic_template = str(weight_topic_template)
        self.task_metrics_topic_template = str(task_metrics_topic_template)
        self.rate_hz = float(max(rate_hz, 0.1))
        self.log_interval_sec = float(max(log_interval_sec, 0.5))
        self.opportunity_top_k = max(1, int(opportunity_top_k))
        self.candidate_distance_max_m = float(candidate_distance_max_m)
        self.overlap_distance_threshold_m = float(max(0.0, overlap_distance_threshold_m))
        self.battery_per_meter = float(max(0.0, battery_per_meter))
        self.battery_return_margin = float(max(0.0, battery_return_margin))
        self.battery_return_reserve = float(max(0.0, battery_return_reserve))
        self.low_battery_threshold = float(max(0.0, low_battery_threshold))
        self.home_pose_path = self._resolve_home_pose_path(home_pose_path)
        self.home_xy_by_robot = _load_home_positions(self.home_pose_path)

        self.global_state = np.zeros((7,), dtype=np.float32)
        self.per_robot_state = np.zeros((self.robot_count * 4,), dtype=np.float32)
        self.frontier_snapshots: List[Dict[str, Any]] = []
        self.charger_status: Dict[str, Any] = {}
        self.energy_metrics_by_id: Dict[int, Dict[str, Any]] = {
            rid: {} for rid in range(self.robot_count)
        }
        self.robot_xy_by_id: Dict[int, Optional[Tuple[float, float]]] = {
            rid: None for rid in range(self.robot_count)
        }
        self.last_global_state_time = 0.0
        self.last_per_robot_state_time = 0.0
        self.last_frontier_time = 0.0
        self.last_odom_time_by_id: Dict[int, float] = {rid: 0.0 for rid in range(self.robot_count)}
        self.last_log_time = 0.0
        self.stop_reason = ''

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        payload = torch.load(str(Path(model_path).expanduser().resolve()), map_location=self.device)
        self.checkpoint_env_config = self._load_checkpoint_env_config(model_path, payload)
        self.local_obs_dim = int(payload['local_obs_dim'])
        self.shared_obs_dim = int(payload['shared_obs_dim'])
        self.hidden_dim = int(payload.get('hidden_dim', 128))
        self.action_mode = _infer_action_mode(payload, action_mode)
        self.policy = MAPPOPolicy(
            local_obs_dim=self.local_obs_dim,
            shared_obs_dim=self.shared_obs_dim,
            hidden_dim=self.hidden_dim,
            action_mode=self.action_mode,
        ).to(self.device)
        self.policy.load_state_dict(payload['policy_state_dict'])
        self.policy.eval()

        self.observation_schema_version = str(payload.get('observation_schema_version', ''))
        self.action_schema_version = str(payload.get('action_schema_version', 'fuzzy_weights_v1'))
        if self.local_obs_dim not in (7, 10, 18):
            raise RuntimeError(
                f'Unsupported MAPPO checkpoint local_obs_dim={self.local_obs_dim}. Expected 7, 10, or 18.'
            )
        if self.action_schema_version not in ('fuzzy_weights_v1', 'fuzzy_weights_simplex_v2'):
            raise RuntimeError(f'Unsupported action schema: {self.action_schema_version}')

        self.logger = MAPPOInferenceLogger(
            model_path=model_path,
            rate_hz=self.rate_hz,
            log_interval_sec=self.log_interval_sec,
            save_enabled=save_enabled,
            save_root=save_root,
            run_name=run_name,
            robot_count=self.robot_count,
            robot_namespace=self.robot_namespace,
        )

        self.state_sub = self.create_subscription(
            Float64MultiArray, self.global_state_topic, self._global_state_cb, 10
        )
        self.per_robot_state_sub = self.create_subscription(
            Float64MultiArray, self.per_robot_state_topic, self._per_robot_state_cb, 10
        )
        self.frontier_sub = self.create_subscription(
            FrontierArrayExtended, self.frontier_topic, self._frontier_cb, 10
        )
        self.charger_status_sub = self.create_subscription(
            String, '/charging_station/status', self._charger_status_cb, 10
        )

        self.weight_pubs = []
        self.weight_topics = []
        self.task_metric_subs = []
        self.task_metric_topics = []
        for rid in range(self.robot_count):
            robot_name = f'{self.robot_namespace}{rid}'
            topic = self.weight_topic_template.format(robot_id=rid, robot_name=robot_name)
            pub = self.create_publisher(Float32MultiArray, topic, 10)
            self.weight_pubs.append(pub)
            self.weight_topics.append(topic)
            task_metrics_topic = self.task_metrics_topic_template.format(robot_id=rid, robot_name=robot_name)
            sub = self.create_subscription(
                String,
                task_metrics_topic,
                lambda msg, name=robot_name: self._task_metrics_cb(name, msg),
                20,
            )
            self.task_metric_subs.append(sub)
            self.task_metric_topics.append(task_metrics_topic)
            self.create_subscription(
                String,
                f'/{robot_name}/low_battery_metrics',
                lambda msg, idx=rid: self._energy_metrics_cb(idx, msg),
                20,
            )
            odom_topic = self.odom_topic_template.format(robot_id=rid, robot_name=robot_name)
            self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, idx=rid: self._odom_cb(idx, msg),
                10,
            )

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f'MAPPO inference ready: model={model_path}, robots={self.robot_count}, '
            f'global_state_topic={self.global_state_topic}, per_robot_state_topic={self.per_robot_state_topic}, '
            f'frontier_topic={self.frontier_topic}, local_obs_dim={self.local_obs_dim}, '
            f'action_mode={self.action_mode}, '
            f'weight_topics={self.weight_topics}, task_metric_topics={self.task_metric_topics}, '
            f'home_pose_path={self.home_pose_path or "(none)"}, '
            f'save_dir={self.logger.run_dir_str() or "(disabled)"}, '
            f'step_metrics={self.logger.step_metrics_path or "(disabled)"}, '
            f'task_metrics={self.logger.task_metrics_path or "(disabled)"}'
        )

    def _resolve_home_pose_path(self, raw_path: str) -> str:
        txt = str(raw_path or '').strip()
        if txt:
            return str(Path(txt).expanduser().resolve())
        if get_package_share_directory is None:
            return ''
        try:
            return str(
                Path(get_package_share_directory('simulation_bringup'))
                / 'config'
                / 'robot_initial_poses.yaml'
            )
        except Exception:
            return ''

    @staticmethod
    def _load_checkpoint_env_config(model_path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        embedded = payload.get('env_config', {})
        if isinstance(embedded, dict) and embedded:
            return dict(embedded)
        model = Path(model_path).expanduser().resolve()
        for candidate in (model.parent / 'config.json', model.parent.parent / 'config.json'):
            if not candidate.is_file():
                continue
            try:
                with candidate.open('r', encoding='utf-8') as stream:
                    raw = json.load(stream)
                env_cfg = raw.get('env_config', {}) if isinstance(raw, dict) else {}
                if isinstance(env_cfg, dict):
                    return dict(env_cfg)
            except Exception:
                continue
        return {}

    def _global_state_cb(self, msg: Float64MultiArray) -> None:
        values = np.asarray(msg.data, dtype=np.float32).reshape(-1)
        if values.size < 7:
            return
        self.global_state = values[:7].copy()
        self.last_global_state_time = time.time()

    def _per_robot_state_cb(self, msg: Float64MultiArray) -> None:
        values = np.asarray(msg.data, dtype=np.float32).reshape(-1)
        expected = self.robot_count * 4
        if values.size < expected:
            return
        self.per_robot_state = values[:expected].copy()
        self.last_per_robot_state_time = time.time()

    def _frontier_cb(self, msg: FrontierArrayExtended) -> None:
        snapshots: List[Dict[str, Any]] = []
        for frontier in msg.frontiers_extended:
            dist_by_robot: Dict[int, float] = {}
            if frontier.robot_names and len(frontier.robot_names) == len(frontier.distances_to_robots):
                name_to_distance = {
                    str(name): float(dist)
                    for name, dist in zip(frontier.robot_names, frontier.distances_to_robots)
                }
                for rid in range(self.robot_count):
                    robot_name = f'{self.robot_namespace}{rid}'
                    dist_by_robot[rid] = float(name_to_distance.get(robot_name, float('inf')))
            else:
                for rid in range(self.robot_count):
                    if rid < len(frontier.distances_to_robots):
                        dist_by_robot[rid] = float(frontier.distances_to_robots[rid])
                    else:
                        dist_by_robot[rid] = float('inf')

            snapshots.append(
                {
                    'id': int(frontier.id),
                    'x': float(frontier.position[0]),
                    'y': float(frontier.position[1]),
                    'semantic': float(frontier.semantic_score),
                    'gain': float(frontier.exploration_gain),
                    'risk': float(frontier.risk_level),
                    'distance_by_robot': dist_by_robot,
                }
            )
        self.frontier_snapshots = snapshots
        self.last_frontier_time = time.time()

    def _odom_cb(self, rid: int, msg: Odometry) -> None:
        self.robot_xy_by_id[int(rid)] = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        self.last_odom_time_by_id[int(rid)] = time.time()

    def _task_metrics_cb(self, robot_name: str, msg: String) -> None:
        self.logger.record_task_event(robot_name=robot_name, raw_payload=msg.data)

    def _charger_status_cb(self, msg: String) -> None:
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.charger_status = value
        except Exception:
            return

    def _energy_metrics_cb(self, rid: int, msg: String) -> None:
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.energy_metrics_by_id[int(rid)] = value
        except Exception:
            return

    def _candidate_frontiers_for_robot(self, rid: int) -> List[Dict[str, Any]]:
        candidates: List[Tuple[float, Dict[str, Any]]] = []
        for frontier in self.frontier_snapshots:
            dist = float(frontier['distance_by_robot'].get(int(rid), float('inf')))
            if not np.isfinite(dist):
                continue
            if self.candidate_distance_max_m > 0.0 and dist > self.candidate_distance_max_m:
                continue
            candidates.append((dist, frontier))
        candidates.sort(key=lambda item: item[0])
        return [item[1] for item in candidates[: self.opportunity_top_k]]

    def _overlap_risk_by_robot(
        self,
        candidate_by_robot: Dict[int, List[Dict[str, Any]]],
    ) -> Dict[int, float]:
        out: Dict[int, float] = {}
        if self.overlap_distance_threshold_m <= 0.0:
            return {rid: 0.0 for rid in candidate_by_robot.keys()}

        for rid, own_frontiers in candidate_by_robot.items():
            total_pairs = 0
            close_pairs = 0
            for other_rid, other_frontiers in candidate_by_robot.items():
                if int(other_rid) == int(rid):
                    continue
                for frontier in own_frontiers:
                    for other_frontier in other_frontiers:
                        total_pairs += 1
                        dist = math.hypot(
                            float(frontier['x']) - float(other_frontier['x']),
                            float(frontier['y']) - float(other_frontier['y']),
                        )
                        if dist <= self.overlap_distance_threshold_m:
                            close_pairs += 1
            out[int(rid)] = float(close_pairs / total_pairs) if total_pairs > 0 else 0.0
        return out

    def _energy_pressure(
        self,
        battery: float,
        cpu: float,
        bandwidth: float,
        candidates: List[Dict[str, Any]],
        rid: int,
    ) -> float:
        available = float(battery) - self.low_battery_threshold
        if available <= 1e-6:
            return 1.0
        distances = [
            float(frontier['distance_by_robot'].get(int(rid), float('inf')))
            for frontier in candidates
        ]
        distances = [value for value in distances if np.isfinite(value) and value >= 0.0]
        if not distances:
            return 1.0
        distance = float(np.mean(distances))
        resource_multiplier = 1.0 + 0.35 * float(cpu) + 0.50 * (1.0 - float(bandwidth))
        expected_cost = distance * self.battery_per_meter * resource_multiplier
        return float(np.clip(expected_cost / available, 0.0, 1.0))

    @staticmethod
    def _candidate_feature_mean(
        candidates: List[Dict[str, Any]],
        key: str,
        *,
        semantic: bool = False,
    ) -> float:
        if not candidates:
            return 0.0
        values = []
        for frontier in candidates:
            raw = float(frontier.get(key, 0.0))
            values.append(_semantic_norm(raw) if semantic else raw)
        return float(np.clip(np.mean(values), 0.0, 1.0))

    def _build_local_obs(self) -> np.ndarray:
        if self.local_obs_dim == 7:
            return _build_local_obs_legacy(self.global_state, self.per_robot_state, self.robot_count)

        candidate_by_robot = {
            rid: self._candidate_frontiers_for_robot(rid)
            for rid in range(self.robot_count)
        }
        overlap_by_robot = self._overlap_risk_by_robot(candidate_by_robot)

        rows: List[np.ndarray] = []
        for rid in range(self.robot_count):
            base = rid * 4
            battery = float(np.clip(self.per_robot_state[base + 0], 0.0, 1.0))
            cpu = float(np.clip(self.per_robot_state[base + 1], 0.0, 1.0))
            bandwidth = float(np.clip(self.per_robot_state[base + 2], 0.0, 1.0))
            candidates = candidate_by_robot.get(int(rid), [])

            base_row = [
                    battery,
                    cpu,
                    bandwidth,
                    self._energy_pressure(battery, cpu, bandwidth, candidates, rid),
                    float(np.clip(self.global_state[3], 0.0, 1.0)),
                    float(np.clip(self.global_state[5], 0.0, 1.0)),
                    self._candidate_feature_mean(candidates, 'semantic', semantic=True),
                    self._candidate_feature_mean(candidates, 'gain', semantic=False),
                    self._candidate_feature_mean(candidates, 'risk', semantic=False),
                    float(np.clip(overlap_by_robot.get(int(rid), 0.0), 0.0, 1.0)),
                ]
            if self.local_obs_dim == 18:
                metric = self.energy_metrics_by_id.get(int(rid), {})
                status = self.charger_status
                distance = float(metric.get('last_estimated_return_distance_m', 0.0))
                wait_time = float(metric.get('predicted_wait_time_sec', 0.0))
                cfg = self.checkpoint_env_config
                speed = max(0.1, float(cfg.get('robot_speed_mps', 0.35)))
                travel_time = max(0.0, distance) / speed
                movement = float(cfg.get('movement_consumption', self.battery_per_meter)) * distance
                base_cost = float(cfg.get('base_consumption', 0.0)) * travel_time
                sensor_cost = float(cfg.get('sensor_consumption', 0.0)) * travel_time
                compute_cost = float(cfg.get('compute_consumption', 0.0)) * cpu * travel_time
                communication_cost = (
                    float(cfg.get('communication_consumption', 0.0))
                    * (1.0 - bandwidth) * travel_time
                )
                required = movement + base_cost + sensor_cost + compute_cost + communication_cost
                trigger = required + float(cfg.get('battery_return_margin', self.battery_return_margin)) \
                    + float(cfg.get('battery_return_reserve', self.battery_return_reserve))
                trigger = max(trigger, float(cfg.get('charge_start_threshold', self.low_battery_threshold)))
                failure_threshold = float(cfg.get('failure_battery_threshold', 0.0))
                available = max(1e-6, battery - failure_threshold)
                return_pressure = float(np.clip(trigger / available, 0.0, 1.0))
                base_row[3] = return_pressure
                idle_rate = (
                    float(cfg.get('base_consumption', 0.00003))
                    + float(cfg.get('sensor_consumption', 0.00006))
                    + float(cfg.get('compute_consumption', 0.0011)) * cpu
                    + float(cfg.get('communication_consumption', 0.00055)) * (1.0 - bandwidth)
                )
                depletion_time = available / max(1e-6, idle_rate)
                depletion_pressure = float(np.clip(
                    1.0 - depletion_time / max(1.0, float(cfg.get('max_mission_time_sec', 600.0)) * 0.25),
                    0.0, 1.0))
                charge_urgency = float(np.clip(
                    0.45 * (1.0 - battery) + 0.35 * return_pressure + 0.20 * depletion_pressure,
                    0.0, 1.0))
                occupied = 1.0 if bool(status.get('occupied', False)) else 0.0
                owner = int(status.get('owner_robot_id', -1))
                queue_len = len(status.get('waiting', []))
                modes = status.get('mode_by_robot', {})
                mode = str(modes.get(str(rid), modes.get(rid, 'EXPLORING')))
                mode_codes = {
                    'EXPLORING': 0.0, 'RETURNING': 0.2, 'WAITING': 0.4,
                    'CHARGING': 0.6, 'READY': 0.8, 'FAILED': 1.0,
                }
                # The online estimate uses the same 120 s normalization as training.
                base_row.extend([
                    float(np.clip(distance / max(1e-6, self.candidate_distance_max_m), 0.0, 1.0)),
                    charge_urgency,
                    occupied,
                    1.0 if rid == owner else 0.0,
                    float(np.clip(queue_len / max(1, self.robot_count), 0.0, 1.0)),
                    float(np.clip(wait_time / 120.0, 0.0, 1.0)),
                    float(np.clip(depletion_time / 120.0, 0.0, 1.0)),
                    float(mode_codes.get(mode, 0.0)),
                ])
            row = np.asarray(base_row, dtype=np.float32)
            rows.append(row)

        return np.stack(rows, axis=0).astype(np.float32)


    def _tick(self) -> None:
        now = time.time()
        if self.last_global_state_time <= 0.0 or self.last_per_robot_state_time <= 0.0:
            return
        if now - self.last_global_state_time > 3.0 or now - self.last_per_robot_state_time > 3.0:
            self.get_logger().warn(
                f'[RL-MAPPO] stale state detected: global_age={now - self.last_global_state_time:.2f}s, '
                f'per_robot_age={now - self.last_per_robot_state_time:.2f}s'
            )
            return

        local_obs = self._build_local_obs()

        infer_t0 = time.perf_counter()
        with torch.no_grad():
            local_tensor = torch.as_tensor(local_obs, dtype=torch.float32, device=self.device)
        actions = self.policy.deterministic_actions(local_tensor).detach().cpu().numpy()
        effective_weights = np.asarray(actions, dtype=np.float32).copy()
        if self.action_schema_version == 'fuzzy_weights_simplex_v2':
            effective_weights = np.clip(effective_weights, 0.0, 1.0)
            totals = np.sum(effective_weights, axis=1, keepdims=True)
            effective_weights = np.divide(
                effective_weights,
                totals,
                out=np.full_like(effective_weights, 0.25),
                where=totals > 1e-9,
            )
        infer_ms = (time.perf_counter() - infer_t0) * 1000.0

        for rid in range(self.robot_count):
            weights = np.asarray(effective_weights[rid], dtype=np.float32).reshape(-1)
            msg = Float32MultiArray()
            msg.data = [float(x) for x in weights.tolist()]
            self.weight_pubs[rid].publish(msg)

        self.logger.record_step(
            global_state=self.global_state,
            per_robot_state=self.per_robot_state,
            local_obs=local_obs,
            weights=np.asarray(effective_weights, dtype=np.float32),
            inference_ms=float(infer_ms),
        )

        if now - self.last_log_time >= self.log_interval_sec:
            self.last_log_time = now
            weights_str = ', '.join(
                f'{self.robot_namespace}{rid}={[round(float(x), 3) for x in actions[rid].tolist()]}'
                for rid in range(self.robot_count)
            )
            local_obs_str = ', '.join(
                f'{self.robot_namespace}{rid}={[round(float(x), 3) for x in local_obs[rid].tolist()]}'
                for rid in range(self.robot_count)
            )
            self.get_logger().info(
                f'[RL-MAPPO] global_state={[round(float(x), 3) for x in self.global_state.tolist()]} '
                f'local_obs: {local_obs_str} weights: {weights_str} infer_ms={infer_ms:.2f}'
            )

    def destroy_node(self) -> bool:
        self.logger.finalize(self.stop_reason or 'node_shutdown', self.global_state)
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description='ROS2 MAPPO online inference')
    parser.add_argument('--model', required=True)
    parser.add_argument('--robot-count', type=int, default=3)
    parser.add_argument('--robot-namespace', default='robot_')
    parser.add_argument('--global-state-topic', default='/rl_training/global_state')
    parser.add_argument('--per-robot-state-topic', default='/rl_training/per_robot_state')
    parser.add_argument('--frontier-topic', default='/frontiers_info')
    parser.add_argument('--odom-topic-template', default='/{robot_name}/odom')
    parser.add_argument('--weight-topic-template', default='/{robot_name}/rl_adjust_weights')
    parser.add_argument('--task-metrics-topic-template', default='/{robot_name}/task_metrics')
    parser.add_argument('--rate', type=float, default=2.0)
    parser.add_argument('--log-interval', type=float, default=2.0)
    parser.add_argument('--save-enabled', type=str, default='true')
    parser.add_argument('--save-root', type=str, default='output/rl_runs')
    parser.add_argument('--run-name', type=str, default='')
    parser.add_argument('--opportunity-top-k', type=int, default=5)
    parser.add_argument('--candidate-distance-max-m', type=float, default=35.0)
    parser.add_argument('--overlap-distance-threshold-m', type=float, default=2.0)
    parser.add_argument('--battery-per-meter', type=float, default=0.003)
    parser.add_argument('--battery-return-margin', type=float, default=0.05)
    parser.add_argument('--battery-return-reserve', type=float, default=0.03)
    parser.add_argument('--low-battery-threshold', type=float, default=0.0)
    parser.add_argument('--home-pose-path', type=str, default='')
    parser.add_argument('--action-mode', type=str, default='auto')
    args, _unknown = parser.parse_known_args()

    save_enabled = _parse_bool(args.save_enabled, default=True)

    rclpy.init()
    node = MAPPOInferenceNode(
        model_path=args.model,
        robot_count=args.robot_count,
        robot_namespace=args.robot_namespace,
        global_state_topic=args.global_state_topic,
        per_robot_state_topic=args.per_robot_state_topic,
        frontier_topic=args.frontier_topic,
        odom_topic_template=args.odom_topic_template,
        weight_topic_template=args.weight_topic_template,
        task_metrics_topic_template=args.task_metrics_topic_template,
        rate_hz=args.rate,
        log_interval_sec=args.log_interval,
        save_enabled=save_enabled,
        save_root=args.save_root,
        run_name=args.run_name,
        opportunity_top_k=args.opportunity_top_k,
        candidate_distance_max_m=args.candidate_distance_max_m,
        overlap_distance_threshold_m=args.overlap_distance_threshold_m,
        battery_per_meter=args.battery_per_meter,
        battery_return_margin=args.battery_return_margin,
        battery_return_reserve=args.battery_return_reserve,
        low_battery_threshold=args.low_battery_threshold,
        home_pose_path=args.home_pose_path,
        action_mode=args.action_mode,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
