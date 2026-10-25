import threading
import time
from typing import Optional

import numpy as np

try:
    import gymnasium as gym
except Exception:
    import gym

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, Int32MultiArray, Empty

try:
    from cure_explore_interfaces.srv import SystemReset
except Exception:
    SystemReset = None

from .obs_builder import build_observation
from .reward_fn import (
    RewardCalculator,
    RewardConfig,
    RobotRewardCalculator,
    RobotRewardConfig,
    ResourceEventPenaltyCalculator,
    ResourceEventPenaltyConfig,
)


class RLEnvironment(gym.Env):
    metadata = {"render_modes": []}
    _rclpy_refcount = 0
    _rclpy_refcount_lock = threading.Lock()

    def __init__(self, config: dict):
        super().__init__()

        self.node_name = str(config.get('node_name', 'rl_training_env'))
        self.global_state_topic = config.get('global_state_topic', '/rl_training/global_state')
        self.per_robot_state_topic = config.get('per_robot_state_topic', '/rl_training/per_robot_state')
        self.weight_topic = config.get('weight_topic', '/rl_training/weights')
        self.dispatch_tick_topic = config.get('dispatch_tick_topic', '/rl_training/dispatch_tick')
        self.reset_service = config.get('reset_service', '/system_reset')
        self.reset_timeout = float(config.get('reset_timeout', 180.0))
        self.reset_retries = max(1, int(config.get('reset_retries', 2)))
        self.reset_retry_interval_sec = max(0.0, float(config.get('reset_retry_interval_sec', 1.0)))
        self.robot_count = int(config.get('robot_count', 3))
        self.step_dt = float(config.get('step_dt', 0.5))
        self.max_steps = int(config.get('max_steps', 100))
        self.coverage_threshold = float(config.get('coverage_threshold', 0.95))
        self.step_mode = str(config.get('step_mode', 'event')).strip().lower()
        self.step_event_topic = str(config.get('step_event_topic', '/rl_training/step_event'))
        self.resource_event_topic = str(config.get('resource_event_topic', '/rl_training/resource_event'))
        self.event_step_timeout = float(config.get('event_step_timeout', 0.0))
        self.event_wait_poll_sec = float(config.get('event_wait_poll_sec', 0.05))
        self.event_dispatch_tick_heartbeat = bool(config.get('event_dispatch_tick_heartbeat', True))
        self.event_dispatch_tick_interval_sec = max(
            0.1, float(config.get('event_dispatch_tick_interval_sec', 1.0))
        )
        self.event_timeout_fallback_step = bool(config.get('event_timeout_fallback_step', False))
        self.episode_stall_timeout = float(config.get('episode_stall_timeout', 300.0))
        self.wait_state_timeout = float(config.get('wait_state_timeout', 15.0))
        self.log_step_interval = int(config.get('log_step_interval', 20))
        self.log_wait_interval_sec = float(config.get('log_wait_interval_sec', 1.0))
        self.global_reward_weight = float(config.get('global_reward_weight', 0.7))
        self.robot_reward_weight = float(config.get('robot_reward_weight', 0.3))

        self.reward_calc = RewardCalculator(RewardConfig(
            w_coverage=float(config.get('w_coverage', 1.0)),
            w_efficiency=float(config.get('w_efficiency', 0.0)),
            w_resource=float(config.get('w_resource', 0.3)),
            w_conflict=float(config.get('w_conflict', 0.2)),
            w_low_battery=float(config.get('w_low_battery', 0.3)),
            low_battery_threshold=float(config.get('low_battery_threshold', 0.2)),
            conflict_threshold=float(config.get('conflict_threshold', 0.3)),
            use_adaptive_weights=bool(config.get('use_adaptive_weights', False)),
            early_stage_threshold=float(config.get('early_stage_threshold', 0.3)),
            late_stage_threshold=float(config.get('late_stage_threshold', 0.8)),
        ))
        self.robot_reward_calcs = [
            RobotRewardCalculator(RobotRewardConfig(
                success_bonus=float(config.get('robot_success_bonus', 0.25)),
                w_resource=float(config.get('robot_w_resource', 0.4)),
                w_low_battery=float(config.get('robot_w_low_battery', 0.3)),
                low_battery_threshold=float(config.get('low_battery_threshold', 0.2)),
            ))
            for _ in range(max(1, self.robot_count))
        ]
        self.resource_event_penalty_calc = ResourceEventPenaltyCalculator(
            ResourceEventPenaltyConfig(
                low_battery_interrupt_penalty=float(config.get('low_battery_interrupt_penalty', 0.4)),
                return_home_failed_penalty=float(config.get('return_home_failed_penalty', 1.0)),
                water_entry_penalty=float(config.get('water_entry_penalty', 1.0)),
            )
        )

        self.action_space = gym.spaces.Box(low=0.0, high=1.0, shape=(4,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=0.0, high=1.0, shape=(7,), dtype=np.float32)

        self._init_ros()
        if self.event_timeout_fallback_step:
            self.node.get_logger().warn(
                "[RL-ENV] event_timeout_fallback_step is deprecated and ignored; "
                "use episode_stall_timeout to truncate episodes."
            )
        self.global_state = None
        self.per_robot_state = []
        self.last_state_time = 0.0
        self.last_per_robot_state_time = 0.0
        self.step_count = 0
        self.episode_count = 0
        self.last_step_log_ts = 0.0
        self.episode_start_wall = 0.0
        self.last_reset_success = False
        self.last_reset_duration_sec = 0.0
        self.last_reset_message = ''
        self._event_lock = threading.Lock()
        self._event_counter_total = 0
        self._processed_step_event_count = 0
        self._last_event_robot_id = -1
        self._resource_event_lock = threading.Lock()
        self._resource_event_seq_total = 0
        self._processed_resource_event_seq = 0
        self._resource_event_counts_total = {
            ResourceEventPenaltyCalculator.LOW_BATTERY_INTERRUPT: 0,
            ResourceEventPenaltyCalculator.RETURN_HOME_FAILED: 0,
            ResourceEventPenaltyCalculator.WATER_ENTRY: 0,
        }
        self._processed_resource_event_counts = dict(self._resource_event_counts_total)
        self._episode_resource_event_counts_start = dict(self._resource_event_counts_total)
        self._ros_registered = False

    def _init_ros(self):
        with self._rclpy_refcount_lock:
            if not rclpy.ok():
                rclpy.init(args=None)
            type(self)._rclpy_refcount += 1
            self._ros_registered = True
        self.node = Node(self.node_name)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.node.create_subscription(
            Float64MultiArray,
            self.global_state_topic,
            self._global_state_cb,
            10,
        )
        self.node.create_subscription(
            Float64MultiArray,
            self.per_robot_state_topic,
            self._per_robot_state_cb,
            10,
        )
        self.node.create_subscription(
            Int32,
            self.step_event_topic,
            self._step_event_cb,
            50,
        )
        self.node.create_subscription(
            Int32MultiArray,
            self.resource_event_topic,
            self._resource_event_cb,
            50,
        )
        self.weight_pub = self.node.create_publisher(Float64MultiArray, self.weight_topic, 10)
        self.dispatch_tick_pub = self.node.create_publisher(Empty, self.dispatch_tick_topic, 10)
        if SystemReset is not None:
            self.reset_client = self.node.create_client(SystemReset, self.reset_service)
        else:
            self.reset_client = None

        self._spin_running = True
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        while self._spin_running and rclpy.ok():
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception:
                if self._spin_running:
                    raise

    def _global_state_cb(self, msg: Float64MultiArray):
        self.global_state = list(msg.data)
        self.last_state_time = time.time()

    def _per_robot_state_cb(self, msg: Float64MultiArray):
        expected = self.robot_count * 4
        if len(msg.data) < expected:
            return
        states = []
        for i in range(self.robot_count):
            base = i * 4
            states.append({
                'battery': float(np.clip(msg.data[base + 0], 0.0, 1.0)),
                'cpu': float(np.clip(msg.data[base + 1], 0.0, 1.0)),
                'bandwidth': float(np.clip(msg.data[base + 2], 0.0, 1.0)),
                'online': float(np.clip(msg.data[base + 3], 0.0, 1.0)),
            })
        self.per_robot_state = states
        self.last_per_robot_state_time = time.time()

    def _step_event_cb(self, msg: Int32):
        with self._event_lock:
            self._event_counter_total += 1
            self._last_event_robot_id = int(msg.data)

    def _resource_event_cb(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            return
        event_type = int(msg.data[0])
        with self._resource_event_lock:
            self._resource_event_seq_total += 1
            self._resource_event_counts_total[event_type] = (
                int(self._resource_event_counts_total.get(event_type, 0)) + 1
            )

    def _publish_weights(self, weights: np.ndarray):
        weights = np.clip(weights, 1e-3, None)
        weights = weights / np.sum(weights)
        msg = Float64MultiArray()
        msg.data = weights.tolist()
        self.weight_pub.publish(msg)

    def _publish_dispatch_tick(self):
        if self.dispatch_tick_pub is None:
            return
        self.dispatch_tick_pub.publish(Empty())

    def _call_reset_once(self):
        if self.reset_client is None:
            return False, 0.0, f"reset service '{self.reset_service}' client unavailable"
        if not self.reset_client.wait_for_service(timeout_sec=self.reset_timeout):
            return False, 0.0, (
                f"reset service '{self.reset_service}' unavailable within "
                f"{self.reset_timeout:.1f}s"
            )

        req = SystemReset.Request()
        req.new_poses = []
        req.reset_simulation = False
        call_start = time.time()
        future = self.reset_client.call_async(req)
        wait_deadline = call_start + self.reset_timeout
        while not future.done() and time.time() < wait_deadline:
            time.sleep(0.05)
        if future.done() and future.result() is not None:
            resp = future.result()
            return bool(resp.success), float(resp.reset_duration_seconds), str(resp.message)
        return False, max(0.0, time.time() - call_start), 'reset service call timeout or failed'

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self.episode_count += 1
        self.reward_calc.reset()
        for calc in self.robot_reward_calcs:
            calc.reset()
        self.resource_event_penalty_calc.reset()
        self.last_step_log_ts = time.time()

        self.node.get_logger().info(
            f"[RL-ENV] Episode {self.episode_count} reset start "
            f"(reset_service={self.reset_service}, coverage_threshold={self.coverage_threshold:.3f})"
        )
        self.last_reset_success = False
        self.last_reset_duration_sec = 0.0
        self.last_reset_message = ''

        # Call reset service with retry (transient readiness/service jitter is common after heavy reset).
        if self.reset_client is not None:
            for attempt in range(1, self.reset_retries + 1):
                ok, duration, message = self._call_reset_once()
                self.last_reset_success = bool(ok)
                self.last_reset_duration_sec = float(duration)
                self.last_reset_message = str(message)
                log_message = (
                    f"[RL-ENV] Reset response (attempt {attempt}/{self.reset_retries}): "
                    f"success={ok}, duration={duration:.3f}s, message='{message}'"
                )
                if ok:
                    self.node.get_logger().info(log_message)
                else:
                    self.node.get_logger().warn(log_message)
                if ok:
                    break
                if attempt < self.reset_retries and self.reset_retry_interval_sec > 0.0:
                    time.sleep(self.reset_retry_interval_sec)

        # Wait for fresh global state to avoid starting from all-zero observation.
        wait_start = time.time()
        next_wait_log = wait_start
        while self.global_state is None and (time.time() - wait_start) < self.wait_state_timeout:
            now = time.time()
            if now >= next_wait_log:
                self.node.get_logger().warn(
                    f"[RL-ENV] Waiting for /rl_training/global_state... "
                    f"elapsed={now - wait_start:.1f}s"
                )
                next_wait_log = now + self.log_wait_interval_sec
            time.sleep(0.05)

        obs = build_observation(self.global_state)
        if self.global_state is None:
            self.node.get_logger().warn(
                f"[RL-ENV] Global state not received after {self.wait_state_timeout:.1f}s, using zero observation"
            )
        else:
            self.node.get_logger().info(
                f"[RL-ENV] Episode {self.episode_count} reset done: "
                f"state=[{', '.join(f'{v:.3f}' for v in obs.tolist())}]"
            )
        self.episode_start_wall = time.time()
        with self._event_lock:
            self._processed_step_event_count = self._event_counter_total
        with self._resource_event_lock:
            self._processed_resource_event_seq = self._resource_event_seq_total
            self._processed_resource_event_counts = dict(self._resource_event_counts_total)
            self._episode_resource_event_counts_start = dict(self._resource_event_counts_total)
        info = {
            'reset_success': 1.0 if self.last_reset_success else 0.0,
            'reset_duration_seconds': float(self.last_reset_duration_sec),
            'reset_message': self.last_reset_message,
        }
        return obs, info

    def step(self, action: np.ndarray):
        self._publish_weights(action)
        self._publish_dispatch_tick()

        step_wait_start = time.time()
        step_event_robot_id = -1
        step_triggered_by_event = False
        step_triggered_by_resource_event = False

        if self.step_mode == 'event':
            next_dispatch_tick = step_wait_start + self.event_dispatch_tick_interval_sec
            while True:
                with self._event_lock:
                    current_count = self._event_counter_total
                    current_robot = self._last_event_robot_id
                with self._resource_event_lock:
                    current_resource_seq = self._resource_event_seq_total
                if current_count > self._processed_step_event_count:
                    step_triggered_by_event = True
                    step_event_robot_id = current_robot
                    break
                if current_resource_seq > self._processed_resource_event_seq:
                    step_triggered_by_resource_event = True
                    break

                now = time.time()
                if self.event_dispatch_tick_heartbeat and now >= next_dispatch_tick:
                    self._publish_dispatch_tick()
                    next_dispatch_tick = now + self.event_dispatch_tick_interval_sec

                if (
                    self.episode_stall_timeout > 0.0 and
                    (now - step_wait_start) >= self.episode_stall_timeout
                ):
                    state_age = now - self.last_state_time if self.last_state_time > 0 else float('inf')
                    per_state_age = (
                        now - self.last_per_robot_state_time
                        if self.last_per_robot_state_time > 0 else float('inf')
                    )
                    self.node.get_logger().warn(
                        f"[RL-ENV] event stall timeout={self.episode_stall_timeout:.1f}s, "
                        f"truncate current episode (step_events={self._processed_step_event_count}->{current_count}, "
                        f"resource_events={self._processed_resource_event_seq}->{current_resource_seq}, "
                        f"state_age={state_age:.2f}s, per_robot_state_age={per_state_age:.2f}s)"
                    )
                    obs = build_observation(self.global_state)
                    info = {
                        'coverage': float(obs[3]) if obs is not None else 0.0,
                        'step_count': self.step_count,
                        'step_mode': self.step_mode,
                        'step_triggered_by_event': 0.0,
                        'step_triggered_by_resource_event': 0.0,
                        'step_event_robot_id': -1.0,
                        'step_wait_seconds': float(time.time() - step_wait_start),
                        'reset_success': 1.0 if self.last_reset_success else 0.0,
                        'reset_duration_seconds': float(self.last_reset_duration_sec),
                        'stall_truncated': 1.0,
                    }
                    return obs, 0.0, False, True, info

                time.sleep(max(0.01, self.event_wait_poll_sec))
        else:
            time.sleep(self.step_dt)

        self.step_count += 1

        obs = build_observation(self.global_state)
        with self._event_lock:
            self._processed_step_event_count = self._event_counter_total
        with self._resource_event_lock:
            current_resource_counts = dict(self._resource_event_counts_total)
            self._processed_resource_event_seq = self._resource_event_seq_total
        new_resource_event_counts = {}
        for event_type, current_count in current_resource_counts.items():
            previous_count = int(self._processed_resource_event_counts.get(event_type, 0))
            new_resource_event_counts[event_type] = max(0, int(current_count) - previous_count)
        self._processed_resource_event_counts = current_resource_counts

        global_reward = self.reward_calc(self.global_state)
        robot_reward = 0.0
        robot_reward_detail = {}
        if (
            0 <= step_event_robot_id < len(self.robot_reward_calcs) and
            len(self.per_robot_state) > step_event_robot_id
        ):
            robot_reward = self.robot_reward_calcs[step_event_robot_id](
                self.per_robot_state[step_event_robot_id]
            )
            robot_reward_detail = self.robot_reward_calcs[step_event_robot_id].get_last_step_detail()
        resource_event_penalty = self.resource_event_penalty_calc(new_resource_event_counts)
        resource_event_detail = self.resource_event_penalty_calc.get_last_step_detail()
        reward = (
            self.global_reward_weight * float(global_reward) +
            self.robot_reward_weight * float(robot_reward) -
            float(resource_event_penalty)
        )
        reward_detail = self.reward_calc.get_last_step_detail()

        coverage = float(obs[3]) if obs is not None else 0.0
        terminated = coverage >= self.coverage_threshold
        truncated = self.step_count >= self.max_steps
        info = {
            'coverage': coverage,
            'step_count': self.step_count,
            'trigger_robot_id': float(step_event_robot_id),
            'global_reward': float(global_reward),
            'robot_reward': float(robot_reward),
            'resource_event_penalty': float(resource_event_penalty),
            'global_reward_weight': float(self.global_reward_weight),
            'robot_reward_weight': float(self.robot_reward_weight),
        }
        info.update(reward_detail)
        info.update(robot_reward_detail)
        info.update(resource_event_detail)

        now = time.time()
        episode_elapsed_sec = max(0.0, now - self.episode_start_wall) if self.episode_start_wall > 0.0 else 0.0
        episode_step_time_sec = float(self.step_count) * float(self.step_dt)
        step_wait_seconds = max(0.0, now - step_wait_start)
        should_log = (
            self.log_step_interval > 0 and
            (self.step_count % self.log_step_interval == 0 or terminated or truncated)
        )
        info.update({
            'episode_elapsed_seconds': episode_elapsed_sec,
            'episode_step_time_seconds': episode_step_time_sec,
            'step_mode': self.step_mode,
            'step_triggered_by_event': 1.0 if step_triggered_by_event else 0.0,
            'step_triggered_by_resource_event': 1.0 if step_triggered_by_resource_event else 0.0,
            'step_event_robot_id': float(step_event_robot_id),
            'step_wait_seconds': float(step_wait_seconds),
            'reset_success': 1.0 if self.last_reset_success else 0.0,
            'reset_duration_seconds': float(self.last_reset_duration_sec),
            'episode_low_battery_interrupt_count': float(
                current_resource_counts.get(ResourceEventPenaltyCalculator.LOW_BATTERY_INTERRUPT, 0) -
                self._episode_resource_event_counts_start.get(ResourceEventPenaltyCalculator.LOW_BATTERY_INTERRUPT, 0)
            ),
            'episode_return_home_failed_count': float(
                current_resource_counts.get(ResourceEventPenaltyCalculator.RETURN_HOME_FAILED, 0) -
                self._episode_resource_event_counts_start.get(ResourceEventPenaltyCalculator.RETURN_HOME_FAILED, 0)
            ),
            'episode_water_entry_count': float(
                current_resource_counts.get(ResourceEventPenaltyCalculator.WATER_ENTRY, 0) -
                self._episode_resource_event_counts_start.get(ResourceEventPenaltyCalculator.WATER_ENTRY, 0)
            ),
        })
        if should_log and (now - self.last_step_log_ts >= 0.1):
            self.last_step_log_ts = now
            state_age = now - self.last_state_time if self.last_state_time > 0 else float('inf')
            self.node.get_logger().info(
                f"[RL-ENV] Ep={self.episode_count} Step={self.step_count} "
                f"reward={reward:.5f} coverage={coverage:.4f} "
                f"battery={obs[0]:.3f} cpu={obs[1]:.3f} bw={obs[2]:.3f} conflict={obs[6]:.3f} "
                f"trigger_robot={step_event_robot_id} "
                f"resource_evt={int(resource_event_detail.get('low_battery_interrupt_count', 0.0) + resource_event_detail.get('return_home_failed_count', 0.0) + resource_event_detail.get('water_entry_count', 0.0))} "
                f"state_age={state_age:.2f}s term={terminated} trunc={truncated}"
            )
            self.node.get_logger().info(
                f"[RL-ENV] Reward detail: global={global_reward:.5f} robot={robot_reward:.5f} "
                f"evt_pen={resource_event_penalty:.5f} "
                f"cov={reward_detail.get('coverage_reward', 0.0):.5f} "
                f"eff={reward_detail.get('efficiency_reward', 0.0):.5f} "
                f"res_pen={reward_detail.get('resource_penalty', 0.0):.5f} "
                f"conf_pen={reward_detail.get('conflict_penalty', 0.0):.5f} "
                f"low_batt_pen={reward_detail.get('low_battery_penalty', 0.0):.5f} "
                f"w=[{reward_detail.get('w_coverage', 0.0):.2f},"
                f"{reward_detail.get('w_efficiency', 0.0):.2f},"
                f"{reward_detail.get('w_resource', 0.0):.2f},"
                f"{reward_detail.get('w_conflict', 0.0):.2f},"
                f"{reward_detail.get('w_low_battery', 0.0):.2f}]"
            )
        return obs, reward, terminated, truncated, info

    def close(self):
        self._spin_running = False
        try:
            if hasattr(self, 'node'):
                self.node.destroy_node()
        finally:
            if hasattr(self, 'executor'):
                try:
                    self.executor.shutdown(timeout_sec=0.5)
                except Exception:
                    pass
            if hasattr(self, '_spin_thread') and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)
            with self._rclpy_refcount_lock:
                if self._ros_registered:
                    type(self)._rclpy_refcount = max(0, type(self)._rclpy_refcount - 1)
                    self._ros_registered = False
                if type(self)._rclpy_refcount == 0 and rclpy.ok():
                    rclpy.shutdown()
