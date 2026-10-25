from dataclasses import dataclass
from typing import Optional, Dict

import numpy as np


@dataclass
class RewardConfig:
    # 基础权重
    w_coverage: float = 1.0
    w_efficiency: float = 0.0
    w_resource: float = 0.3
    w_conflict: float = 0.2
    w_low_battery: float = 0.3

    # 阈值
    low_battery_threshold: float = 0.2
    conflict_threshold: float = 0.3

    # 自适应权重（探索阶段）
    use_adaptive_weights: bool = False
    early_stage_threshold: float = 0.3
    late_stage_threshold: float = 0.8


@dataclass
class RobotRewardConfig:
    # 成功完成一次任务时的基础奖励
    success_bonus: float = 0.25
    # 机器人资源惩罚
    w_resource: float = 0.4
    # 低电量惩罚
    w_low_battery: float = 0.3
    low_battery_threshold: float = 0.2


@dataclass
class ResourceEventPenaltyConfig:
    low_battery_interrupt_penalty: float = 0.4
    return_home_failed_penalty: float = 1.0
    water_entry_penalty: float = 1.0


class RewardCalculator:
    """
    改进版奖励函数：
    1) 覆盖率增量奖励（主项）
    2) 探索效率奖励（覆盖率/时间）
    3) 资源消耗惩罚
    4) 冲突惩罚（非线性）
    5) 低电量惩罚
    """

    def __init__(self, config: RewardConfig):
        self.cfg = config
        self.last_coverage = None
        self.last_battery = None
        self.last_conflict = None
        self.step_count = 0
        self.total_reward = 0.0
        self.reward_components = {
            'coverage': 0.0,
            'efficiency': 0.0,
            'resource': 0.0,
            'conflict': 0.0,
            'low_battery': 0.0,
        }
        self.last_step_detail = {}

    def reset(self):
        self.last_coverage = None
        self.last_battery = None
        self.last_conflict = None
        self.step_count = 0
        self.total_reward = 0.0
        self.reward_components = {
            'coverage': 0.0,
            'efficiency': 0.0,
            'resource': 0.0,
            'conflict': 0.0,
            'low_battery': 0.0,
        }
        self.last_step_detail = {}

    def __call__(self, global_state, info: Optional[Dict] = None) -> float:
        # global_state = [avg_battery, avg_cpu, avg_bandwidth, coverage, online, frontier, conflict]
        if global_state is None or len(global_state) < 7:
            return 0.0

        avg_battery = float(global_state[0])
        avg_cpu = float(global_state[1])
        avg_bandwidth = float(global_state[2])
        coverage = float(global_state[3])
        conflict = float(global_state[6])

        self.step_count += 1
        weights = self._get_adaptive_weights(coverage)

        coverage_reward = self._compute_coverage_reward(coverage)
        efficiency_reward = self._compute_efficiency_reward(coverage)
        resource_penalty = self._compute_resource_penalty(avg_battery, avg_cpu, avg_bandwidth)
        conflict_penalty = self._compute_conflict_penalty(conflict)
        low_battery_penalty = self._compute_low_battery_penalty(avg_battery)

        self.last_coverage = coverage
        self.last_battery = avg_battery
        self.last_conflict = conflict

        reward = (
            weights['coverage'] * coverage_reward
            + weights['efficiency'] * efficiency_reward
            - weights['resource'] * resource_penalty
            - weights['conflict'] * conflict_penalty
            - weights['low_battery'] * low_battery_penalty
        )

        self.last_step_detail = {
            'coverage_reward': float(coverage_reward),
            'efficiency_reward': float(efficiency_reward),
            'resource_penalty': float(resource_penalty),
            'conflict_penalty': float(conflict_penalty),
            'low_battery_penalty': float(low_battery_penalty),
            'w_coverage': float(weights['coverage']),
            'w_efficiency': float(weights['efficiency']),
            'w_resource': float(weights['resource']),
            'w_conflict': float(weights['conflict']),
            'w_low_battery': float(weights['low_battery']),
            'reward': float(reward),
            'coverage': float(coverage),
            'battery': float(avg_battery),
            'cpu': float(avg_cpu),
            'bandwidth': float(avg_bandwidth),
            'conflict': float(conflict),
        }

        self.reward_components['coverage'] += coverage_reward
        self.reward_components['efficiency'] += efficiency_reward
        self.reward_components['resource'] += resource_penalty
        self.reward_components['conflict'] += conflict_penalty
        self.reward_components['low_battery'] += low_battery_penalty
        self.total_reward += reward

        return float(reward)

    def _get_adaptive_weights(self, coverage: float) -> Dict[str, float]:
        if not self.cfg.use_adaptive_weights:
            return {
                'coverage': self.cfg.w_coverage,
                'efficiency': self.cfg.w_efficiency,
                'resource': self.cfg.w_resource,
                'conflict': self.cfg.w_conflict,
                'low_battery': self.cfg.w_low_battery,
            }

        if coverage < self.cfg.early_stage_threshold:
            return {
                'coverage': self.cfg.w_coverage * 1.5,
                'efficiency': self.cfg.w_efficiency * 1.3,
                'resource': self.cfg.w_resource * 0.7,
                'conflict': self.cfg.w_conflict * 0.8,
                'low_battery': self.cfg.w_low_battery,
            }
        if coverage > self.cfg.late_stage_threshold:
            return {
                'coverage': self.cfg.w_coverage * 0.8,
                'efficiency': self.cfg.w_efficiency * 0.7,
                'resource': self.cfg.w_resource * 1.3,
                'conflict': self.cfg.w_conflict * 1.5,
                'low_battery': self.cfg.w_low_battery * 1.2,
            }
        return {
            'coverage': self.cfg.w_coverage,
            'efficiency': self.cfg.w_efficiency,
            'resource': self.cfg.w_resource,
            'conflict': self.cfg.w_conflict,
            'low_battery': self.cfg.w_low_battery,
        }

    def _compute_coverage_reward(self, coverage: float) -> float:
        if self.last_coverage is None:
            return 0.0
        return max(coverage - self.last_coverage, 0.0)

    def _compute_efficiency_reward(self, coverage: float) -> float:
        if self.last_coverage is None or self.step_count <= 0:
            return 0.0
        delta = max(coverage - self.last_coverage, 0.0)
        return float(delta / max(self.step_count, 1))

    def _compute_resource_penalty(self, battery: float, cpu: float, bandwidth: float) -> float:
        # battery 消耗 + 资源压力
        battery_consumption = 1.0 - battery
        resource_pressure = (1.0 - bandwidth) * 0.5 + cpu * 0.5
        penalty = battery_consumption * 1.0 + resource_pressure
        return float(np.clip(penalty, 0.0, 2.0))

    def _compute_conflict_penalty(self, conflict_rate: float) -> float:
        if conflict_rate < self.cfg.conflict_threshold:
            return 0.0
        excess = conflict_rate - self.cfg.conflict_threshold
        penalty = (excess / (1.0 - self.cfg.conflict_threshold)) ** 2
        return float(penalty)

    def _compute_low_battery_penalty(self, battery: float) -> float:
        return 1.0 if battery <= self.cfg.low_battery_threshold else 0.0

    def get_statistics(self) -> Dict[str, float]:
        if self.step_count == 0:
            return {}
        return {
            'total_reward': self.total_reward,
            'avg_reward': self.total_reward / self.step_count,
            'coverage_reward': self.reward_components['coverage'],
            'efficiency_reward': self.reward_components['efficiency'],
            'resource_penalty': self.reward_components['resource'],
            'conflict_penalty': self.reward_components['conflict'],
            'low_battery_penalty': self.reward_components['low_battery'],
            'step_count': self.step_count,
        }

    def get_last_step_detail(self) -> Dict:
        return dict(self.last_step_detail)

    def print_statistics(self):
        stats = self.get_statistics()
        if not stats:
            print("无统计信息")
            return
        print("\n" + "=" * 50)
        print("奖励统计")
        print("=" * 50)
        print(f"总奖励: {stats['total_reward']:.2f}")
        print(f"平均奖励: {stats['avg_reward']:.4f}")
        print(f"步数: {stats['step_count']}")
        print("\n奖励成分:")
        print(f"  覆盖率奖励: {stats['coverage_reward']:.2f}")
        print(f"  效率奖励: {stats['efficiency_reward']:.2f}")
        print(f"  资源惩罚: {stats['resource_penalty']:.2f}")
        print(f"  冲突惩罚: {stats['conflict_penalty']:.2f}")
        print(f"  低电量惩罚: {stats['low_battery_penalty']:.2f}")
        print("=" * 50 + "\n")


class RobotRewardCalculator:
    """
    单机器人奖励计算器（每个机器人独立维护一份）。
    仅在“该机器人成功完成任务”事件时调用一次。
    """

    def __init__(self, config: RobotRewardConfig):
        self.cfg = config
        self.step_count = 0
        self.total_reward = 0.0
        self.last_step_detail = {}

    def reset(self):
        self.step_count = 0
        self.total_reward = 0.0
        self.last_step_detail = {}

    def __call__(self, robot_state: Optional[Dict]) -> float:
        # robot_state: {'battery':0~1, 'cpu':0~1, 'bandwidth':0~1, 'online':0/1}
        self.step_count += 1
        if robot_state is None:
            self.last_step_detail = {
                'robot_reward': 0.0,
                'robot_resource_penalty': 0.0,
                'robot_low_battery_penalty': 0.0,
                'robot_success_bonus': 0.0,
                'robot_state_missing': 1.0,
            }
            return 0.0

        battery = float(np.clip(robot_state.get('battery', 1.0), 0.0, 1.0))
        cpu = float(np.clip(robot_state.get('cpu', 0.0), 0.0, 1.0))
        bandwidth = float(np.clip(robot_state.get('bandwidth', 1.0), 0.0, 1.0))
        online = float(np.clip(robot_state.get('online', 1.0), 0.0, 1.0))

        # 探索成功时给基础奖励，但离线状态不应获得奖励。
        success_bonus = self.cfg.success_bonus * online
        resource_penalty = float(np.clip((1.0 - battery) + 0.5 * cpu + 0.5 * (1.0 - bandwidth), 0.0, 2.0))
        low_battery_penalty = 1.0 if battery <= self.cfg.low_battery_threshold else 0.0

        reward = (
            success_bonus
            - self.cfg.w_resource * resource_penalty
            - self.cfg.w_low_battery * low_battery_penalty
        )

        self.total_reward += float(reward)
        self.last_step_detail = {
            'robot_reward': float(reward),
            'robot_resource_penalty': float(resource_penalty),
            'robot_low_battery_penalty': float(low_battery_penalty),
            'robot_success_bonus': float(success_bonus),
            'robot_battery': float(battery),
            'robot_cpu': float(cpu),
            'robot_bandwidth': float(bandwidth),
            'robot_online': float(online),
            'robot_state_missing': 0.0,
        }
        return float(reward)

    def get_last_step_detail(self) -> Dict:
        return dict(self.last_step_detail)


class ResourceEventPenaltyCalculator:
    """
    资源相关事件惩罚：
    1) 任务执行途中低电量中断并返航
    2) 返航失败
    """

    LOW_BATTERY_INTERRUPT = 1
    RETURN_HOME_FAILED = 2
    WATER_ENTRY = 3

    def __init__(self, config: ResourceEventPenaltyConfig):
        self.cfg = config
        self.last_step_detail = {}

    def reset(self):
        self.last_step_detail = {}

    def __call__(self, event_counts: Optional[Dict[int, int]]) -> float:
        event_counts = event_counts or {}
        low_battery_interrupt_count = int(event_counts.get(self.LOW_BATTERY_INTERRUPT, 0))
        return_home_failed_count = int(event_counts.get(self.RETURN_HOME_FAILED, 0))
        water_entry_count = int(event_counts.get(self.WATER_ENTRY, 0))

        low_battery_interrupt_penalty = (
            float(low_battery_interrupt_count) * float(self.cfg.low_battery_interrupt_penalty)
        )
        return_home_failed_penalty = (
            float(return_home_failed_count) * float(self.cfg.return_home_failed_penalty)
        )
        water_entry_penalty = (
            float(water_entry_count) * float(self.cfg.water_entry_penalty)
        )
        total_penalty = (
            low_battery_interrupt_penalty +
            return_home_failed_penalty +
            water_entry_penalty
        )

        self.last_step_detail = {
            'resource_event_penalty': float(total_penalty),
            'low_battery_interrupt_count': float(low_battery_interrupt_count),
            'return_home_failed_count': float(return_home_failed_count),
            'water_entry_count': float(water_entry_count),
            'low_battery_interrupt_penalty': float(low_battery_interrupt_penalty),
            'return_home_failed_penalty': float(return_home_failed_penalty),
            'water_entry_penalty': float(water_entry_penalty),
        }
        return float(total_penalty)

    def get_last_step_detail(self) -> Dict:
        return dict(self.last_step_detail)
