#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from frontier_msgs.msg import FrontierArrayExtended
from fuzzy_msgs.msg import FuzzyPriority, FuzzyPriorityArray
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String

try:
    import torch
except Exception:
    torch = None

from .bandwidth_mre_model import PolicyNet
from .bandwidth_mre_runtime import FrontierPoint, OnlineLocalGraphBuilder, RuntimeCommunicationStore


class JsonlInferenceLogger:
    def __init__(self, *, save_enabled: bool, save_root: str, run_name: str) -> None:
        self.enabled = bool(save_enabled)
        self.run_dir: Optional[Path] = None
        self.step_metrics_path: Optional[Path] = None
        self.task_metrics_path: Optional[Path] = None
        self.low_battery_metrics_path: Optional[Path] = None
        self._step_fp = None
        self._task_fp = None
        self._battery_fp = None
        if not self.enabled:
            return

        root = Path(save_root or "output/rl_runs").expanduser()
        name = (run_name or "bandwidth_mre_infer").strip()
        run_dir = root / name
        if run_dir.exists():
            run_dir = root / f"{name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        logs_dir = run_dir / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)
        self.run_dir = run_dir
        self.step_metrics_path = logs_dir / "step_metrics.jsonl"
        self.task_metrics_path = logs_dir / "task_metrics.jsonl"
        self.low_battery_metrics_path = logs_dir / "low_battery_metrics.jsonl"
        self._step_fp = open(self.step_metrics_path, "a", encoding="utf-8", buffering=1)
        self._task_fp = open(self.task_metrics_path, "a", encoding="utf-8", buffering=1)
        self._battery_fp = open(self.low_battery_metrics_path, "a", encoding="utf-8", buffering=1)

    def close(self) -> None:
        for fp in (self._step_fp, self._task_fp, self._battery_fp):
            if fp is not None:
                fp.close()

    @staticmethod
    def _write(fp, payload: Dict) -> None:
        if fp is not None:
            fp.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")

    def write_step(self, payload: Dict) -> None:
        self._write(self._step_fp, payload)

    def write_task_event(self, payload: Dict) -> None:
        self._write(self._task_fp, payload)

    def write_low_battery(self, payload: Dict) -> None:
        self._write(self._battery_fp, payload)


def _parse_bool(raw: str, default: bool = True) -> bool:
    txt = str(raw or "").strip().lower()
    if txt in {"1", "true", "yes", "y", "on"}:
        return True
    if txt in {"0", "false", "no", "n", "off"}:
        return False
    return bool(default)


def _select_device(requested: str):
    if torch is None:
        raise RuntimeError("PyTorch is not available. Activate the conda env that contains torch.")
    req = str(requested or "auto").lower()
    if req == "cpu":
        return torch.device("cpu")
    if req == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError("--device cuda requested, but CUDA is not available")
        return torch.device("cuda")
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def _canonical_policy_state_dict(state):
    """Accept checkpoints saved with the paper's original module names.

    The trained Bandwidth-MRE checkpoint was produced from an implementation
    whose encoder/decoder blocks use names such as ``multiHeadAttention`` and
    ``normalization1``.  The local inference model is structurally identical,
    but uses shorter Pythonic names.  Remapping here keeps inference
    self-contained without depending on the training-side source tree.
    """
    replacements = (
        (".multiHeadAttention.", ".attn."),
        (".normalization1.", ".norm1."),
        (".normalization2.", ".norm2."),
        (".feedForward.", ".ff."),
    )
    out = {}
    converted = 0
    for key, value in state.items():
        new_key = str(key)
        for old, new in replacements:
            if old in new_key:
                new_key = new_key.replace(old, new)
        if new_key != key:
            converted += 1
        out[new_key] = value
    return out, converted


class BandwidthMREInferenceNode(Node):
    def __init__(
        self,
        *,
        model_path: str,
        robot_count: int,
        robot_namespace: str,
        global_map_topic: str,
        frontier_topic: str,
        odom_topic_template: str,
        task_topic_template: str,
        debug_priority_topic_template: str,
        rate_hz: float,
        device: str,
        local_radius_m: float,
        decision_min_distance_m: float,
        decision_max_distance_m: float,
        sample_spacing_m: float,
        edge_radius_m: float,
        utility_radius_m: float,
        utility_top_k: int,
        guidepost_top_k: int,
        coord_norm_scale_m: float,
        target_hold_sec: float,
        goal_reached_tolerance_m: float,
        obstacle_threshold: int,
        startup_warmup_sec: float,
        require_all_robots_ready: bool,
        log_interval_sec: float,
        save_enabled: bool,
        save_root: str,
        run_name: str,
        task_metrics_topic_template: str,
        low_battery_metrics_topic_template: str,
    ) -> None:
        super().__init__("rl_infer_bandwidth_mre")
        if torch is None:
            raise RuntimeError("PyTorch is not available. Activate the conda env that contains torch.")

        self.robot_count = int(robot_count)
        self.robot_ids = list(range(self.robot_count))
        self.robot_namespace = str(robot_namespace)
        self.global_map_topic = str(global_map_topic)
        self.frontier_topic = str(frontier_topic)
        self.odom_topic_template = str(odom_topic_template)
        self.task_topic_template = str(task_topic_template)
        self.debug_priority_topic_template = str(debug_priority_topic_template)
        self.rate_hz = max(0.1, float(rate_hz))
        self.target_hold_sec = max(0.0, float(target_hold_sec))
        self.goal_reached_tolerance_m = max(0.05, float(goal_reached_tolerance_m))
        self.startup_warmup_sec = max(0.0, float(startup_warmup_sec))
        self.require_all_robots_ready = bool(require_all_robots_ready)
        self.warmup_started = False
        self.warmup_completed = self.startup_warmup_sec <= 0.0
        self.warmup_deadline = 0.0
        self.warmup_wait_logged = False
        self.log_interval_sec = max(0.5, float(log_interval_sec))
        self.last_log_time = 0.0
        self.start_wall_time = time.time()
        self.last_task_by_robot: Dict[int, Optional[Dict[str, float]]] = {rid: None for rid in self.robot_ids}
        self.last_priorities_by_robot: Dict[int, List[Dict[str, float]]] = {rid: [] for rid in self.robot_ids}
        self.last_task_events: Dict[str, Dict] = {}
        self.last_low_battery: Dict[str, Dict] = {}
        self.logger = JsonlInferenceLogger(
            save_enabled=save_enabled,
            save_root=save_root,
            run_name=run_name,
        )

        payload = torch.load(str(Path(model_path).expanduser().resolve()), map_location="cpu")
        cfg = dict(payload.get("config", {}))
        self.node_feature_dim = int(cfg.get("node_feature_dim", 5))
        self.node_padding_size = int(cfg.get("node_padding_size", 360))
        self.max_neighbor_actions = int(cfg.get("max_neighbor_actions", 25))
        self.embedding_dim = int(cfg.get("embedding_dim", 64))
        self.message_dim = int(cfg.get("message_dim", self.embedding_dim))
        if self.message_dim != self.embedding_dim:
            raise RuntimeError(
                f"message_dim must match embedding_dim for PolicyNet runtime: "
                f"{self.message_dim} != {self.embedding_dim}"
            )

        self.device = _select_device(device)
        self.policy = PolicyNet(self.node_feature_dim, self.embedding_dim).to(self.device)
        state = payload.get("policy") or payload.get("policy_state_dict")
        if state is None:
            raise RuntimeError(f"model checkpoint has no policy state: {model_path}")
        state, converted_keys = _canonical_policy_state_dict(state)
        self.policy.load_state_dict(state)
        self.policy.eval()

        self.builder = OnlineLocalGraphBuilder(
            node_padding_size=self.node_padding_size,
            max_neighbor_actions=self.max_neighbor_actions,
            local_radius_m=local_radius_m,
            decision_min_distance_m=decision_min_distance_m,
            decision_max_distance_m=decision_max_distance_m,
            sample_spacing_m=sample_spacing_m,
            edge_radius_m=edge_radius_m,
            utility_radius_m=utility_radius_m,
            utility_top_k=utility_top_k,
            guidepost_top_k=guidepost_top_k,
            coord_norm_scale_m=coord_norm_scale_m,
            obstacle_threshold=obstacle_threshold,
        )
        self.comm = RuntimeCommunicationStore(self.robot_ids, self.message_dim)

        self.global_map: Optional[OccupancyGrid] = None
        self.frontiers: List[FrontierPoint] = []
        self.robot_xy: Dict[int, Optional[Tuple[float, float]]] = {rid: None for rid in self.robot_ids}
        self.robot_failed: Dict[int, bool] = {rid: False for rid in self.robot_ids}
        self.last_target_xy: Dict[int, Optional[Tuple[float, float]]] = {rid: None for rid in self.robot_ids}
        self.last_target_time: Dict[int, float] = {rid: 0.0 for rid in self.robot_ids}

        self.create_subscription(OccupancyGrid, self.global_map_topic, self._map_cb, 10)
        self.create_subscription(FrontierArrayExtended, self.frontier_topic, self._frontier_cb, 10)
        for rid in self.robot_ids:
            robot_name = f"{self.robot_namespace}{rid}"
            topic = self.odom_topic_template.format(robot_name=robot_name, robot_id=rid)
            self.create_subscription(Odometry, topic, lambda msg, idx=rid: self._odom_cb(idx, msg), 10)
            self.create_subscription(
                Bool,
                f"/{robot_name}/failed",
                lambda msg, idx=rid: self._failed_cb(idx, msg),
                10,
            )
            task_metrics_topic = task_metrics_topic_template.format(robot_name=robot_name, robot_id=rid)
            low_battery_topic = low_battery_metrics_topic_template.format(robot_name=robot_name, robot_id=rid)
            self.create_subscription(
                String,
                task_metrics_topic,
                lambda msg, name=robot_name: self._task_metrics_cb(name, msg),
                20,
            )
            self.create_subscription(
                String,
                low_battery_topic,
                lambda msg, name=robot_name: self._low_battery_metrics_cb(name, msg),
                10,
            )

        self.task_pubs = {}
        self.debug_priority_pubs = {}
        for rid in self.robot_ids:
            robot_name = f"{self.robot_namespace}{rid}"
            task_topic = self.task_topic_template.format(robot_name=robot_name, robot_id=rid)
            debug_topic = self.debug_priority_topic_template.format(robot_name=robot_name, robot_id=rid)
            self.task_pubs[rid] = self.create_publisher(FuzzyPriority, task_topic, 10)
            self.debug_priority_pubs[rid] = self.create_publisher(FuzzyPriorityArray, debug_topic, 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            "Bandwidth-MRE inference started: "
            f"robots={self.robot_count}, device={self.device}, node_padding={self.node_padding_size}, "
            f"actions={self.max_neighbor_actions}, local_radius={local_radius_m:.1f}, "
            f"decision_range=[{decision_min_distance_m:.1f},{decision_max_distance_m:.1f}], "
            f"utility_radius={utility_radius_m:.1f}, utility_top_k={utility_top_k}, "
            f"guidepost_top_k={guidepost_top_k}, startup_warmup={self.startup_warmup_sec:.1f}s, "
            f"require_all_ready={self.require_all_robots_ready}, converted_checkpoint_keys={converted_keys}"
        )
        if self.logger.enabled:
            self.get_logger().info(
                "Bandwidth-MRE inference logging enabled: "
                f"run_dir={self.logger.run_dir}, step_metrics={self.logger.step_metrics_path}, "
                f"task_metrics={self.logger.task_metrics_path}, low_battery={self.logger.low_battery_metrics_path}"
            )

    def destroy_node(self):
        self.logger.close()
        super().destroy_node()

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.global_map = msg

    def _odom_cb(self, rid: int, msg: Odometry) -> None:
        self.robot_xy[int(rid)] = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def _frontier_cb(self, msg: FrontierArrayExtended) -> None:
        out: List[FrontierPoint] = []
        for item in msg.frontiers_extended:
            try:
                out.append(
                    FrontierPoint(
                        frontier_id=int(item.id),
                        x=float(item.position[0]),
                        y=float(item.position[1]),
                        gain=float(item.exploration_gain),
                        risk=float(item.risk_level),
                        semantic=float(item.semantic_score),
                    )
                )
            except Exception:
                continue
        self.frontiers = out

    def _task_metrics_cb(self, robot_name: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"raw": msg.data}
        if not isinstance(payload, dict):
            payload = {"raw": msg.data}
        record = {
            "timestamp": time.time(),
            "elapsed_sec": time.time() - self.start_wall_time,
            "robot_name": robot_name,
            **payload,
        }
        self.last_task_events[robot_name] = record
        self.logger.write_task_event(record)

    def _low_battery_metrics_cb(self, robot_name: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"raw": msg.data}
        if not isinstance(payload, dict):
            payload = {"raw": msg.data}
        record = {
            "timestamp": time.time(),
            "elapsed_sec": time.time() - self.start_wall_time,
            "robot_name": robot_name,
            **payload,
        }
        self.last_low_battery[robot_name] = record
        self.logger.write_low_battery(record)

    def _other_robot_xy(self, rid: int) -> List[Tuple[float, float]]:
        out = []
        for other_id, xy in self.robot_xy.items():
            if int(other_id) == int(rid) or xy is None:
                continue
            out.append((float(xy[0]), float(xy[1])))
        return out

    def _build_obs(self, rid: int):
        xy = self.robot_xy.get(int(rid))
        if xy is None or self.global_map is None:
            return None
        return self.builder.build(
            torch_module=torch,
            map_msg=self.global_map,
            robot_xy=xy,
            other_robot_xy=self._other_robot_xy(rid),
            frontiers=self.frontiers,
            device=self.device,
        )

    def _robots_ready_for_startup(self) -> bool:
        if self.global_map is None or not self.frontiers:
            return False
        if not self.require_all_robots_ready:
            return any(self.robot_xy[rid] is not None for rid in self.robot_ids)
        return all(self.robot_xy[rid] is not None for rid in self.robot_ids)

    def _startup_warmup_satisfied(self, now_sec: float) -> bool:
        if self.warmup_completed:
            return True
        if not self._robots_ready_for_startup():
            if not self.warmup_wait_logged:
                self.get_logger().info(
                    "Bandwidth-MRE waiting for startup readiness: "
                    "global_map, non-empty frontiers, and robot odometry"
                )
                self.warmup_wait_logged = True
            return False
        if not self.warmup_started:
            self.warmup_started = True
            self.warmup_deadline = float(now_sec) + self.startup_warmup_sec
            self.get_logger().info(
                f"Bandwidth-MRE startup warmup started: wait {self.startup_warmup_sec:.1f}s before publishing tasks"
            )
            return False
        if float(now_sec) < self.warmup_deadline:
            return False
        self.warmup_completed = True
        self.get_logger().info("Bandwidth-MRE startup warmup finished: task publishing enabled")
        return True

    def _update_messages(self, now_sec: float) -> None:
        with torch.no_grad():
            for rid in self.robot_ids:
                obs = self._build_obs(rid)
                if obs is None:
                    continue
                _, feature = self.policy.get_current_state_feature(
                    obs.node_inputs,
                    obs.node_padding_mask,
                    obs.edge_mask,
                    obs.current_index,
                    obs.current_coord,
                )
                self.comm.write(int(rid), feature.detach().cpu().numpy().reshape(-1), now_sec)

    def _target_still_active(self, rid: int, now_sec: float) -> bool:
        target = self.last_target_xy.get(int(rid))
        robot = self.robot_xy.get(int(rid))
        if target is None or robot is None:
            return False
        if self.target_hold_sec > 0.0 and now_sec - float(self.last_target_time.get(int(rid), 0.0)) > self.target_hold_sec:
            return False
        dist = float(np.linalg.norm(np.asarray(target, dtype=np.float32) - np.asarray(robot, dtype=np.float32)))
        return dist > self.goal_reached_tolerance_m

    def _make_priority(self, *, rid: int, idx: int, xy: Tuple[float, float], score: float) -> FuzzyPriority:
        msg = FuzzyPriority()
        msg.frontier_id = int(1_000_000 + int(rid) * 10_000 + int(idx))
        msg.total_score = float(score)
        msg.weight_extremely_high = float(score)
        msg.weight_high = 0.0
        msg.weight_medium = 0.0
        msg.weight_low = 0.0
        p = Point()
        p.x = float(xy[0])
        p.y = float(xy[1])
        p.z = 0.0
        msg.global_position = p
        msg.local_position = p
        msg.transform_successful = True
        return msg

    def _publish_debug_priorities(self, rid: int, priorities: List[FuzzyPriority]) -> None:
        arr = FuzzyPriorityArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = "global_map"
        arr.priorities = priorities
        self.debug_priority_pubs[int(rid)].publish(arr)

    def _publish_task(self, rid: int, task: FuzzyPriority) -> None:
        if self.robot_failed.get(int(rid), False):
            return
        self.task_pubs[int(rid)].publish(task)

    def _failed_cb(self, rid: int, msg: Bool) -> None:
        self.robot_failed[int(rid)] = bool(msg.data)
        if msg.data:
            self.last_target_xy[int(rid)] = None

    @staticmethod
    def _priority_to_dict(item: FuzzyPriority) -> Dict[str, float]:
        return {
            "frontier_id": int(item.frontier_id),
            "score": float(item.total_score),
            "x": float(item.global_position.x),
            "y": float(item.global_position.y),
        }

    def _infer_robot_task(self, rid: int, now_sec: float) -> Tuple[Optional[FuzzyPriority], List[FuzzyPriority]]:
        if self._target_still_active(rid, now_sec):
            return None, []

        obs = self._build_obs(rid)
        if obs is None or len(obs.candidate_xy) <= 1:
            return None, []

        msg_stacked_np = self.comm.read_stacked(self.robot_ids)
        msg_stacked = torch.as_tensor(msg_stacked_np, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            logp = self.policy(
                obs.node_inputs,
                obs.node_padding_mask,
                obs.edge_mask,
                obs.current_index,
                obs.current_edge,
                obs.edge_padding_mask,
                obs.current_coord,
                msg_stacked,
            )
            probs = logp.exp().detach().cpu().numpy().reshape(-1)

        priorities: List[FuzzyPriority] = []
        for idx, xy in enumerate(obs.candidate_xy):
            if idx == 0:
                continue
            score = float(probs[idx]) if idx < probs.shape[0] else 0.0
            score = score + 0.05 * float(obs.candidate_utility[idx])
            if score <= 0.0:
                continue
            priorities.append(self._make_priority(rid=rid, idx=idx, xy=xy, score=score))
        priorities.sort(key=lambda item: item.total_score, reverse=True)
        if not priorities:
            return None, []
        best = priorities[0]
        self.last_target_xy[int(rid)] = (float(best.global_position.x), float(best.global_position.y))
        self.last_target_time[int(rid)] = float(now_sec)
        return best, priorities

    def _write_step_metrics(
        self,
        *,
        now_sec: float,
        active_count: int,
        published_tasks: Dict[int, FuzzyPriority],
    ) -> None:
        robots = []
        for rid in self.robot_ids:
            robot_name = f"{self.robot_namespace}{rid}"
            robot_xy = self.robot_xy.get(rid)
            target_xy = self.last_target_xy.get(rid)
            task = published_tasks.get(rid)
            if task is not None:
                self.last_task_by_robot[rid] = self._priority_to_dict(task)
            robots.append(
                {
                    "robot_id": int(rid),
                    "robot_name": robot_name,
                    "x": float(robot_xy[0]) if robot_xy is not None else None,
                    "y": float(robot_xy[1]) if robot_xy is not None else None,
                    "target_x": float(target_xy[0]) if target_xy is not None else None,
                    "target_y": float(target_xy[1]) if target_xy is not None else None,
                    "published_task": self._priority_to_dict(task) if task is not None else None,
                    "last_task": self.last_task_by_robot.get(rid),
                    "top_priorities": self.last_priorities_by_robot.get(rid, [])[:5],
                    "last_task_event": self.last_task_events.get(robot_name),
                    "last_low_battery": self.last_low_battery.get(robot_name),
                }
            )
        self.logger.write_step(
            {
                "timestamp": float(now_sec),
                "elapsed_sec": float(now_sec - self.start_wall_time),
                "frontier_count": int(len(self.frontiers)),
                "active_published_tasks": int(active_count),
                "map_ready": self.global_map is not None,
                "robots_ready": all(self.robot_xy[rid] is not None for rid in self.robot_ids),
                "robots": robots,
            }
        )

    def _tick(self) -> None:
        now_sec = time.time()
        if self.global_map is None or not self.frontiers:
            return
        if any(
            self.robot_xy[rid] is None
            for rid in self.robot_ids
            if not self.robot_failed.get(rid, False)
        ):
            return
        if not self._startup_warmup_satisfied(now_sec):
            return

        self._update_messages(now_sec)
        active = 0
        published_tasks: Dict[int, FuzzyPriority] = {}
        for rid in self.robot_ids:
            if self.robot_failed.get(rid, False):
                continue
            task, priorities = self._infer_robot_task(rid, now_sec)
            self._publish_debug_priorities(rid, priorities)
            self.last_priorities_by_robot[rid] = [self._priority_to_dict(item) for item in priorities[:10]]
            if task is not None:
                active += 1
                published_tasks[int(rid)] = task
                self._publish_task(rid, task)

        self._write_step_metrics(now_sec=now_sec, active_count=active, published_tasks=published_tasks)

        if now_sec - self.last_log_time >= self.log_interval_sec:
            self.last_log_time = now_sec
            self.get_logger().info(
                f"[Bandwidth-MRE] frontiers={len(self.frontiers)} active={active}/{self.robot_count}"
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--robot-count", type=int, default=3)
    parser.add_argument("--robot-namespace", default="robot_")
    parser.add_argument("--global-map-topic", default="/global_map")
    parser.add_argument("--frontier-topic", default="/frontiers_info")
    parser.add_argument("--odom-topic-template", default="/{robot_name}/odom")
    parser.add_argument("--task-topic-template", default="/{robot_name}/task_point")
    parser.add_argument("--debug-priority-topic-template", default="/{robot_name}/bandwidth_mre_priority_debug")
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--device", choices=["auto", "cpu", "cuda"], default="auto")
    parser.add_argument("--local-radius-m", type=float, default=12.0)
    parser.add_argument("--decision-min-distance-m", type=float, default=3.0)
    parser.add_argument("--decision-max-distance-m", type=float, default=9.0)
    parser.add_argument("--sample-spacing-m", type=float, default=1.5)
    parser.add_argument("--edge-radius-m", type=float, default=2.5)
    parser.add_argument("--utility-radius-m", type=float, default=18.0)
    parser.add_argument("--utility-top-k", type=int, default=5)
    parser.add_argument("--guidepost-top-k", type=int, default=5)
    parser.add_argument("--coord-norm-scale-m", type=float, default=40.0)
    parser.add_argument("--target-hold-sec", type=float, default=8.0)
    parser.add_argument("--goal-reached-tolerance-m", type=float, default=1.0)
    parser.add_argument("--obstacle-threshold", type=int, default=50)
    parser.add_argument("--startup-warmup-sec", type=float, default=8.0)
    parser.add_argument("--require-all-robots-ready", default="true")
    parser.add_argument("--log-interval", type=float, default=2.0)
    parser.add_argument("--save-enabled", default="true")
    parser.add_argument("--save-root", default="output/rl_runs")
    parser.add_argument("--run-name", default="bandwidth_mre_infer")
    parser.add_argument("--task-metrics-topic-template", default="/{robot_name}/task_metrics")
    parser.add_argument("--low-battery-metrics-topic-template", default="/{robot_name}/low_battery_metrics")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = BandwidthMREInferenceNode(
        model_path=args.model,
        robot_count=args.robot_count,
        robot_namespace=args.robot_namespace,
        global_map_topic=args.global_map_topic,
        frontier_topic=args.frontier_topic,
        odom_topic_template=args.odom_topic_template,
        task_topic_template=args.task_topic_template,
        debug_priority_topic_template=args.debug_priority_topic_template,
        rate_hz=args.rate,
        device=args.device,
        local_radius_m=args.local_radius_m,
        decision_min_distance_m=args.decision_min_distance_m,
        decision_max_distance_m=args.decision_max_distance_m,
        sample_spacing_m=args.sample_spacing_m,
        edge_radius_m=args.edge_radius_m,
        utility_radius_m=args.utility_radius_m,
        utility_top_k=args.utility_top_k,
        guidepost_top_k=args.guidepost_top_k,
        coord_norm_scale_m=args.coord_norm_scale_m,
        target_hold_sec=args.target_hold_sec,
        goal_reached_tolerance_m=args.goal_reached_tolerance_m,
        obstacle_threshold=args.obstacle_threshold,
        startup_warmup_sec=args.startup_warmup_sec,
        require_all_robots_ready=_parse_bool(args.require_all_robots_ready, default=True),
        log_interval_sec=args.log_interval,
        save_enabled=_parse_bool(args.save_enabled, default=True),
        save_root=args.save_root,
        run_name=args.run_name,
        task_metrics_topic_template=args.task_metrics_topic_template,
        low_battery_metrics_topic_template=args.low_battery_metrics_topic_template,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
