#!/usr/bin/env python3
"""
状态验证器：检查 7 维全局状态的合法性与异常变化
"""

from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple

import numpy as np


@dataclass
class ValidationConfig:
    min_value: float = 0.0
    max_value: float = 1.0
    enable_outlier_detection: bool = True
    outlier_std_threshold: float = 3.0
    enable_change_rate_check: bool = True
    max_change_rate: float = 0.5
    min_valid_ratio: float = 0.8


class StateValidator:
    def __init__(self, config: Optional[ValidationConfig] = None):
        self.cfg = config or ValidationConfig()
        self.history: List[np.ndarray] = []
        self.max_history_size = 100
        self.total_validations = 0
        self.invalid_count = 0
        self.outlier_count = 0
        self.rapid_change_count = 0

    def validate(self, state: np.ndarray, verbose: bool = False) -> Tuple[bool, Optional[str]]:
        self.total_validations += 1
        if state is None:
            return False, "状态为 None"
        if len(state) != 7:
            return False, f"状态维度错误: {len(state)} (期望 7)"
        if not np.all(np.isfinite(state)):
            self.invalid_count += 1
            return False, "状态包含 NaN 或 Inf"
        if not np.all((state >= self.cfg.min_value) & (state <= self.cfg.max_value)):
            self.invalid_count += 1
            return False, f"状态值超出范围 [{self.cfg.min_value}, {self.cfg.max_value}]"

        if self.cfg.enable_outlier_detection and len(self.history) > 10:
            is_outlier, outlier_msg = self._check_outliers(state)
            if is_outlier:
                self.outlier_count += 1
                if verbose:
                    return True, f"警告: {outlier_msg}"

        if self.cfg.enable_change_rate_check and len(self.history) > 0:
            is_rapid, change_msg = self._check_change_rate(state)
            if is_rapid:
                self.rapid_change_count += 1
                if verbose:
                    return True, f"警告: {change_msg}"

        self.history.append(state.copy())
        if len(self.history) > self.max_history_size:
            self.history.pop(0)
        return True, None

    def _check_outliers(self, state: np.ndarray) -> Tuple[bool, Optional[str]]:
        history_array = np.array(self.history)
        mean = np.mean(history_array, axis=0)
        std = np.std(history_array, axis=0)
        std = np.maximum(std, 1e-6)
        z_scores = np.abs((state - mean) / std)
        outlier_mask = z_scores > self.cfg.outlier_std_threshold
        if np.any(outlier_mask):
            dim_names = ['battery', 'cpu', 'bandwidth', 'coverage', 'online', 'frontier', 'conflict']
            outlier_dims = np.where(outlier_mask)[0]
            outlier_names = [dim_names[i] for i in outlier_dims]
            return True, f"异常值检测: {outlier_names}"
        return False, None

    def _check_change_rate(self, state: np.ndarray) -> Tuple[bool, Optional[str]]:
        last_state = self.history[-1]
        change = np.abs(state - last_state)
        rapid_change_mask = change > self.cfg.max_change_rate
        if np.any(rapid_change_mask):
            dim_names = ['battery', 'cpu', 'bandwidth', 'coverage', 'online', 'frontier', 'conflict']
            rapid_dims = np.where(rapid_change_mask)[0]
            rapid_names = [dim_names[i] for i in rapid_dims]
            max_change = np.max(change)
            return True, f"快速变化: {rapid_names} (最大变化: {max_change:.3f})"
        return False, None

    def get_statistics(self) -> Dict:
        if self.total_validations == 0:
            return {}
        return {
            'total_validations': self.total_validations,
            'invalid_count': self.invalid_count,
            'outlier_count': self.outlier_count,
            'rapid_change_count': self.rapid_change_count,
            'invalid_ratio': self.invalid_count / self.total_validations,
            'outlier_ratio': self.outlier_count / self.total_validations,
            'rapid_change_ratio': self.rapid_change_count / self.total_validations,
        }

    def print_statistics(self):
        stats = self.get_statistics()
        if not stats:
            print("无验证统计信息")
            return
        print("\n" + "=" * 50)
        print("状态验证统计")
        print("=" * 50)
        print(f"总验证次数: {stats['total_validations']}")
        print(f"无效次数: {stats['invalid_count']} ({stats['invalid_ratio']*100:.1f}%)")
        print(f"异常值次数: {stats['outlier_count']} ({stats['outlier_ratio']*100:.1f}%)")
        print(f"快速变化次数: {stats['rapid_change_count']} ({stats['rapid_change_ratio']*100:.1f}%)")
        print("=" * 50)
