#!/usr/bin/env python3
import csv
import json
import math
import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrajectorySnapshotRecorder(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_snapshot_recorder")

        self.declare_parameter("robot_count", 3)
        self.declare_parameter("robot_namespace", "robot_")
        self.declare_parameter("global_map_topic", "/global_map")
        self.declare_parameter("summary_path", "")
        self.declare_parameter("output_root", "")
        self.declare_parameter("snapshot_times_sec", "100,200,300,400,500,600,700,800,900,1000")
        self.declare_parameter("check_period_sec", 0.5)
        self.declare_parameter("trajectory_min_step_m", 0.02)
        self.declare_parameter("trajectory_sample_period_sec", 0.5)
        self.declare_parameter("stop_topic", "/exploration_stop")
        self.declare_parameter("low_battery_metrics_topic_template", "/{robot_name}/low_battery_metrics")
        self.declare_parameter("task_metrics_topic_template", "/{robot_name}/task_metrics")

        self.robot_count = int(self.get_parameter("robot_count").value)
        self.robot_namespace = str(self.get_parameter("robot_namespace").value)
        self.global_map_topic = str(self.get_parameter("global_map_topic").value)
        self.summary_path = str(self.get_parameter("summary_path").value)
        self.output_root = str(self.get_parameter("output_root").value)
        self.check_period_sec = float(self.get_parameter("check_period_sec").value)
        self.trajectory_min_step_m = float(self.get_parameter("trajectory_min_step_m").value)
        self.trajectory_sample_period_sec = float(self.get_parameter("trajectory_sample_period_sec").value)
        self.stop_topic = str(self.get_parameter("stop_topic").value)
        self.low_battery_metrics_topic_template = str(
            self.get_parameter("low_battery_metrics_topic_template").value
        )
        self.task_metrics_topic_template = str(self.get_parameter("task_metrics_topic_template").value)

        snapshot_raw = str(self.get_parameter("snapshot_times_sec").value).strip()
        self.snapshot_times_sec: List[float] = []
        for token in snapshot_raw.split(","):
            token = token.strip()
            if not token:
                continue
            try:
                value = float(token)
            except ValueError:
                continue
            if value > 0.0:
                self.snapshot_times_sec.append(value)
        self.snapshot_times_sec = sorted(set(self.snapshot_times_sec))
        self.next_snapshot_index = 0

        base_output = Path(self.output_root).expanduser() if self.output_root else None
        if not base_output:
            if self.summary_path:
                base_output = (
                    Path(self.summary_path).expanduser().resolve().parent
                    / "artifacts"
                    / "trajectory_snapshots"
                )
            else:
                base_output = Path(os.getcwd()) / "data" / "trajectory_snapshots"
        self.output_root_path = base_output
        self.maps_dir = self.output_root_path / "maps"
        self.traj_dir = self.output_root_path / "trajectories"
        self.status_path = self.output_root_path / "snapshot_status.json"
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        self.traj_dir.mkdir(parents=True, exist_ok=True)

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_receive_count = 0
        self.first_map_logged = False
        self.final_snapshot_saved = False
        self.motion_started = False
        self.start_time_mono = time.monotonic()
        self.motion_start_time_mono: Optional[float] = None
        self.goal_settle_wait_time_sec_by_robot: Dict[int, float] = {
            rid: 0.0 for rid in range(self.robot_count)
        }
        self.goal_settle_wait_start_mono: Dict[int, Optional[float]] = {
            rid: None for rid in range(self.robot_count)
        }
        self.goal_settle_wait_intervals: List[Tuple[float, float]] = []
        self.stalled_wait_time_sec = 0.0
        self.last_timer_mono: Optional[float] = None
        self.last_map_receive_count = 0

        self.last_xy: Dict[int, Optional[Tuple[float, float]]] = {rid: None for rid in range(self.robot_count)}
        self.latest_pose: Dict[int, Optional[Tuple[float, float, float]]] = {
            rid: None for rid in range(self.robot_count)
        }
        self.last_trajectory_write_mono: Dict[int, Optional[float]] = {
            rid: None for rid in range(self.robot_count)
        }
        self.csv_files = {}
        self.csv_writers = {}

        for rid in range(self.robot_count):
            robot_name = f"{self.robot_namespace}{rid}"
            csv_path = self.traj_dir / f"{robot_name}.csv"
            fp = open(csv_path, "w", encoding="utf-8", newline="")
            writer = csv.DictWriter(
                fp,
                fieldnames=[
                    "elapsed_sec",
                    "wall_elapsed_sec",
                    "robot_id",
                    "robot_name",
                    "x",
                    "y",
                    "yaw",
                ],
            )
            writer.writeheader()
            self.csv_files[rid] = fp
            self.csv_writers[rid] = writer

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(OccupancyGrid, self.global_map_topic, self.map_callback, map_qos)
        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, self.stop_topic, self.stop_callback, stop_qos)
        for rid in range(self.robot_count):
            robot_name = f"{self.robot_namespace}{rid}"
            self.create_subscription(
                Odometry,
                f"/{robot_name}/odom",
                lambda msg, rid=rid: self.odom_callback(rid, msg),
                20,
            )
            self.create_subscription(
                String,
                self.low_battery_metrics_topic_template.format(robot_id=rid, robot_name=robot_name),
                lambda msg, rid=rid: self.low_battery_callback(rid, msg),
                10,
            )
            self.create_subscription(
                String,
                self.task_metrics_topic_template.format(robot_id=rid, robot_name=robot_name),
                lambda msg, rid=rid: self.task_metrics_callback(rid, msg),
                20,
            )

        self.timer = self.create_timer(self.check_period_sec, self.timer_callback)
        self._write_status("started")
        self.get_logger().info(
            "TrajectorySnapshotRecorder started: "
            f"summary={self.summary_path or '<none>'}, map_topic={self.global_map_topic}, "
            f"output={self.output_root_path}, snapshots={self.snapshot_times_sec}"
        )

    def destroy_node(self):
        try:
            self._save_final_snapshot()
        except Exception as ex:
            self.get_logger().warn(f"Failed to save final snapshot during shutdown: {ex}")
        for fp in self.csv_files.values():
            try:
                fp.close()
            except Exception:
                pass
        super().destroy_node()

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_receive_count += 1
        if not self.first_map_logged:
            self.first_map_logged = True
            self._write_status("first_map_received")
            self.get_logger().info(
                "Received first global_map: "
                f"{msg.info.width}x{msg.info.height} res={msg.info.resolution:.3f}"
            )

    def _elapsed_sec(self) -> float:
        return self._wall_elapsed_sec()

    def _wall_elapsed_sec(self) -> float:
        if self.motion_started and self.motion_start_time_mono is not None:
            return max(0.0, time.monotonic() - self.motion_start_time_mono)
        return 0.0

    def low_battery_callback(self, rid: int, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        self.goal_settle_wait_time_sec_by_robot[rid] = float(payload.get("goal_settle_wait_time_sec", 0.0))

    def task_metrics_callback(self, rid: int, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        event = str(payload.get("event", ""))
        now = time.monotonic()
        if event == "task_settle_wait_started":
            if self.goal_settle_wait_start_mono[rid] is None:
                self.goal_settle_wait_start_mono[rid] = now
        elif event == "task_settle_wait_finished":
            start = self.goal_settle_wait_start_mono[rid]
            if start is not None and now >= start:
                self.goal_settle_wait_intervals.append((float(start), float(now)))
            self.goal_settle_wait_start_mono[rid] = None

    @staticmethod
    def _merged_interval_length(intervals: List[Tuple[float, float]]) -> float:
        clean: List[Tuple[float, float]] = []
        for start, end in intervals:
            if end > start:
                clean.append((float(start), float(end)))
        if not clean:
            return 0.0
        clean.sort()
        cur_start, cur_end = clean[0]
        total = 0.0
        for start, end in clean[1:]:
            if start <= cur_end:
                cur_end = max(cur_end, end)
            else:
                total += cur_end - cur_start
                cur_start, cur_end = start, end
        total += cur_end - cur_start
        return total

    def _global_goal_settle_wait_time_sec(self) -> float:
        now = time.monotonic()
        intervals = list(self.goal_settle_wait_intervals)
        for rid in range(self.robot_count):
            start = self.goal_settle_wait_start_mono.get(rid)
            if start is not None and now > start:
                intervals.append((float(start), float(now)))
        return self._merged_interval_length(intervals)

    def _any_robot_wait_time_sec(self) -> float:
        return self._global_goal_settle_wait_time_sec()

    def _all_robots_wait_time_sec(self) -> float:
        now = time.monotonic()
        events: List[Tuple[float, int]] = []
        intervals = list(self.goal_settle_wait_intervals)
        for rid in range(self.robot_count):
            start = self.goal_settle_wait_start_mono.get(rid)
            if start is not None and now > start:
                intervals.append((float(start), float(now)))
        for start, end in intervals:
            if end <= start:
                continue
            events.append((float(start), 1))
            events.append((float(end), -1))
        if not events:
            return 0.0
        events.sort(key=lambda item: (item[0], -item[1]))
        active = 0
        prev_t: Optional[float] = None
        total = 0.0
        for t, delta in events:
            if prev_t is not None and t > prev_t and active >= self.robot_count:
                total += t - prev_t
            active += delta
            prev_t = t
        return total

    def _update_stalled_wait_accumulator(self) -> None:
        if not self.motion_started:
            return
        now = time.monotonic()
        if self.last_timer_mono is None:
            self.last_timer_mono = now
            self.last_map_receive_count = self.map_receive_count
            return
        dt = max(0.0, now - self.last_timer_mono)
        any_waiting = any(start is not None for start in self.goal_settle_wait_start_mono.values())
        map_progress = self.map_receive_count > self.last_map_receive_count
        if any_waiting and not map_progress:
            self.stalled_wait_time_sec += dt
        self.last_timer_mono = now
        self.last_map_receive_count = self.map_receive_count

    def _write_status(self, state: str, extra: Optional[Dict[str, object]] = None) -> None:
        payload: Dict[str, object] = {
            "state": state,
            "summary_path": self.summary_path,
            "output_root": str(self.output_root_path),
            "map_topic": self.global_map_topic,
            "map_receive_count": self.map_receive_count,
            "motion_started": self.motion_started,
            "elapsed_sec": float(f"{self._elapsed_sec():.3f}"),
            "wall_elapsed_sec": float(f"{self._wall_elapsed_sec():.3f}"),
            "stalled_wait_time_sec": float(f"{self.stalled_wait_time_sec:.3f}"),
            "any_robot_wait_time_sec": float(f"{self._any_robot_wait_time_sec():.3f}"),
            "all_robots_wait_time_sec": float(f"{self._all_robots_wait_time_sec():.3f}"),
            "snapshot_times_sec": self.snapshot_times_sec,
        }
        if self.map_msg is not None:
            payload["map_info"] = {
                "width": int(self.map_msg.info.width),
                "height": int(self.map_msg.info.height),
                "resolution": float(self.map_msg.info.resolution),
            }
        if extra:
            payload.update(extra)
        self.status_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def odom_callback(self, rid: int, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        self.latest_pose[rid] = (x, y, yaw)

        prev = self.last_xy[rid]

        if prev is not None and not self.motion_started:
            if math.hypot(x - prev[0], y - prev[1]) > 1e-5:
                self.motion_started = True
                self.motion_start_time_mono = time.monotonic()
                self.get_logger().info(f"Robot {rid} first motion detected, snapshot timer started.")

        self.last_xy[rid] = (x, y)

    def _write_trajectory_sample(self, rid: int) -> None:
        pose = self.latest_pose.get(rid)
        if pose is None:
            return
        x, y, yaw = pose
        elapsed = self._elapsed_sec()
        wall_elapsed = self._wall_elapsed_sec()
        writer = self.csv_writers[rid]
        writer.writerow(
            {
                "elapsed_sec": f"{elapsed:.3f}",
                "wall_elapsed_sec": f"{wall_elapsed:.3f}",
                "robot_id": rid,
                "robot_name": f"{self.robot_namespace}{rid}",
                "x": f"{x:.6f}",
                "y": f"{y:.6f}",
                "yaw": f"{yaw:.6f}",
            }
        )
        self.csv_files[rid].flush()

    def _write_due_trajectory_samples(self) -> None:
        if not self.motion_started:
            return
        now = time.monotonic()
        period = max(0.01, self.trajectory_sample_period_sec)
        for rid in range(self.robot_count):
            last_write = self.last_trajectory_write_mono.get(rid)
            if last_write is not None and now - last_write < period:
                continue
            self._write_trajectory_sample(rid)
            self.last_trajectory_write_mono[rid] = now

    def _save_pgm(self, map_msg: OccupancyGrid, image_path: Path) -> None:
        width = int(map_msg.info.width)
        height = int(map_msg.info.height)
        data = list(map_msg.data)
        image_path.parent.mkdir(parents=True, exist_ok=True)
        with open(image_path, "wb") as f:
            f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            pixels = bytearray()
            for gy in range(height - 1, -1, -1):
                row = gy * width
                for gx in range(width):
                    val = int(data[row + gx])
                    if val < 0:
                        px = 205
                    elif val >= 65:
                        px = 0
                    else:
                        px = 254
                    pixels.append(px)
            f.write(pixels)

    def _save_snapshot(self, snap_time_sec: float) -> None:
        if self.map_msg is None:
            self._write_status(
                "snapshot_skipped_no_map",
                {"requested_snapshot_sec": float(f"{snap_time_sec:.3f}")},
            )
            self.get_logger().warn(f"Snapshot t={snap_time_sec:.1f}s skipped: no global_map yet.")
            return

        stamp = f"t{int(round(snap_time_sec)):04d}"
        image_path = self.maps_dir / f"global_map_{stamp}.pgm"
        meta_path = self.maps_dir / f"global_map_{stamp}.json"
        self._save_pgm(self.map_msg, image_path)

        meta = {
            "elapsed_sec": float(f"{snap_time_sec:.3f}"),
            "wall_elapsed_sec": float(f"{self._wall_elapsed_sec():.3f}"),
            "map_topic": self.global_map_topic,
            "width": int(self.map_msg.info.width),
            "height": int(self.map_msg.info.height),
            "resolution": float(self.map_msg.info.resolution),
            "origin": {
                "x": float(self.map_msg.info.origin.position.x),
                "y": float(self.map_msg.info.origin.position.y),
                "z": float(self.map_msg.info.origin.position.z),
            },
            "image_path": str(image_path),
        }
        meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
        self._write_status(
            "snapshot_saved",
            {
                "requested_snapshot_sec": float(f"{snap_time_sec:.3f}"),
                "image_path": str(image_path),
            },
        )
        self.get_logger().info(f"Saved snapshot {stamp}: {image_path.name}")

    def _save_final_snapshot(self) -> None:
        if self.final_snapshot_saved or self.map_msg is None:
            if self.map_msg is None:
                self._write_status("final_snapshot_skipped_no_map")
                self.get_logger().warn("Final snapshot skipped: no global_map received.")
            return
        elapsed = self._elapsed_sec()
        wall_elapsed = self._wall_elapsed_sec()
        image_path = self.maps_dir / "global_map_final.pgm"
        meta_path = self.maps_dir / "global_map_final.json"
        self._save_pgm(self.map_msg, image_path)
        meta = {
            "elapsed_sec": float(f"{elapsed:.3f}"),
            "wall_elapsed_sec": float(f"{wall_elapsed:.3f}"),
            "map_topic": self.global_map_topic,
            "width": int(self.map_msg.info.width),
            "height": int(self.map_msg.info.height),
            "resolution": float(self.map_msg.info.resolution),
            "origin": {
                "x": float(self.map_msg.info.origin.position.x),
                "y": float(self.map_msg.info.origin.position.y),
                "z": float(self.map_msg.info.origin.position.z),
            },
            "image_path": str(image_path),
        }
        meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
        self.final_snapshot_saved = True
        self._write_status("final_snapshot_saved", {"image_path": str(image_path)})
        self.get_logger().info(f"Saved final snapshot: {image_path.name}")

    def stop_callback(self, msg: Bool) -> None:
        if bool(msg.data):
            self._save_final_snapshot()

    def timer_callback(self) -> None:
        self._update_stalled_wait_accumulator()
        self._write_due_trajectory_samples()
        if not self.motion_started:
            return
        elapsed = self._elapsed_sec()
        while self.next_snapshot_index < len(self.snapshot_times_sec):
            target = self.snapshot_times_sec[self.next_snapshot_index]
            if elapsed + 1e-6 < target:
                break
            self._save_snapshot(target)
            self.next_snapshot_index += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectorySnapshotRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
