#!/usr/bin/env python3
"""
训练监控器：TensorBoard + 日志文件 + 训练报告
"""

import json
import time
from pathlib import Path
from typing import Dict, Optional, Any

try:
    from torch.utils.tensorboard import SummaryWriter
    TENSORBOARD_AVAILABLE = True
except Exception:
    TENSORBOARD_AVAILABLE = False


class TrainingMonitor:
    def __init__(
        self,
        log_dir: str = 'logs',
        tensorboard_dir: Optional[str] = None,
        save_interval: int = 10,
        enable_tensorboard: bool = True,
        resume: bool = False,
    ):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.enable_tensorboard = enable_tensorboard and TENSORBOARD_AVAILABLE
        if self.enable_tensorboard:
            tb_dir = tensorboard_dir or str(self.log_dir / 'tensorboard')
            self.writer = SummaryWriter(tb_dir)
        else:
            self.writer = None

        self.save_interval = save_interval
        self.global_step = 0
        self.episode_count = 0
        self.start_time = time.time()
        self.previous_total_time_sec = 0.0
        self.metrics_history = []
        self.evaluation_history = []
        self.best_reward = float('-inf')
        self.best_coverage = 0.0
        self.best_eval_reward = float('-inf')
        self.best_eval_coverage = 0.0
        self.resume = bool(resume)

        self.log_file = self.log_dir / 'training.log'
        self.eval_log_file = self.log_dir / 'evaluation.log'
        self.combined_metrics_path = self.log_dir / 'training_data.json'
        self.metrics_file = self.log_dir / 'metrics.json'
        self.eval_metrics_file = self.log_dir / 'evaluation_metrics.json'

        if self.resume and self.combined_metrics_path.exists():
            self._load_combined_data()
        self._prepare_log_files()
        print(
            f"[RL-MONITOR] log_dir={self.log_dir}, tensorboard={'on' if self.enable_tensorboard else 'off'}, "
            f"resume={'on' if self.resume else 'off'}, episodes={self.episode_count}, steps={self.global_step}"
        )
        self._save_combined_data()

    def _append_log_banner(self, path: Path, title: str, resumed: bool):
        mode = 'a' if resumed else 'w'
        with open(path, mode) as f:
            f.write("=" * 80 + "\n")
            f.write(f"{title}\n")
            f.write(
                f"{'恢复时间' if resumed else '开始时间'}: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            )
            f.write("=" * 80 + "\n\n")

    def _prepare_log_files(self):
        resumed = self.resume and self.combined_metrics_path.exists()
        if not resumed:
            self._append_log_banner(self.log_file, "训练日志", resumed=False)
            self._append_log_banner(self.eval_log_file, "评估日志", resumed=False)
            return

        self._append_log_banner(self.log_file, "恢复训练日志", resumed=True)
        self._append_log_banner(self.eval_log_file, "恢复评估日志", resumed=True)

    def _load_combined_data(self):
        with open(self.combined_metrics_path, 'r') as f:
            data = json.load(f)

        meta = data.get('meta', {})
        summary = data.get('summary', {})
        self.metrics_history = list(data.get('episodes', []))
        self.evaluation_history = list(data.get('evaluations', []))
        self.episode_count = int(meta.get('episode_count', len(self.metrics_history)))
        self.global_step = int(meta.get('global_step', 0))
        self.best_reward = float(summary.get('best_reward', self.best_reward))
        self.best_coverage = float(summary.get('best_coverage', self.best_coverage))
        self.best_eval_reward = float(summary.get('best_eval_reward', self.best_eval_reward))
        self.best_eval_coverage = float(summary.get('best_eval_coverage', self.best_eval_coverage))
        self.previous_total_time_sec = float(summary.get('total_time_sec', 0.0))

    @staticmethod
    def _episode_index_from_item(item: Any, fallback: int) -> int:
        if not isinstance(item, dict):
            return int(fallback)

        value = item.get('episode', None)
        if isinstance(value, (int, float)):
            return int(value)
        if isinstance(value, str):
            try:
                return int(float(value))
            except Exception:
                return int(fallback)
        if isinstance(value, dict):
            for key in ('index', 'episode', 'id', 'num', 'value'):
                nested = value.get(key, None)
                if isinstance(nested, (int, float)):
                    return int(nested)
                if isinstance(nested, str):
                    try:
                        return int(float(nested))
                    except Exception:
                        continue
        return int(fallback)

    def apply_resume_base(
        self,
        episode_base: Optional[int] = None,
        global_step_base: Optional[int] = None,
        truncate_history: bool = True,
    ):
        """
        在续训时强制设置计数基线。
        典型场景：从较早 checkpoint 继续训练，但复用同一个 log_dir。
        """
        if episode_base is None and global_step_base is None:
            return

        target_episode = self.episode_count if episode_base is None else max(0, int(episode_base))
        target_step = self.global_step if global_step_base is None else max(0, int(global_step_base))

        if truncate_history:
            self.metrics_history = [
                item for idx, item in enumerate(self.metrics_history, start=1)
                if self._episode_index_from_item(
                    item,
                    fallback=idx,
                ) <= target_episode
            ]
            self.evaluation_history = [
                item for idx, item in enumerate(self.evaluation_history, start=1)
                if self._episode_index_from_item(
                    item,
                    fallback=idx,
                ) <= target_episode
            ]

            if self.metrics_history:
                self.best_reward = max(float(item.get('reward', float('-inf'))) for item in self.metrics_history)
                self.best_coverage = max(float(item.get('coverage', 0.0)) for item in self.metrics_history)
            else:
                self.best_reward = float('-inf')
                self.best_coverage = 0.0

            if self.evaluation_history:
                self.best_eval_reward = max(float(item.get('mean_reward', float('-inf'))) for item in self.evaluation_history)
                self.best_eval_coverage = max(float(item.get('mean_coverage', 0.0)) for item in self.evaluation_history)
            else:
                self.best_eval_reward = float('-inf')
                self.best_eval_coverage = 0.0

        self.episode_count = target_episode
        self.global_step = target_step
        self._save_combined_data()
        print(
            f"[RL-MONITOR] resume base applied: episodes={self.episode_count}, "
            f"steps={self.global_step}, truncate_history={truncate_history}"
        )

    def _to_jsonable(self, value):
        if value is None or isinstance(value, (bool, int, float, str)):
            return value

        # numpy scalar -> python scalar
        if hasattr(value, 'item'):
            try:
                return value.item()
            except Exception:
                pass

        # numpy array / tensor-like -> list
        if hasattr(value, 'tolist'):
            try:
                return value.tolist()
            except Exception:
                pass

        if isinstance(value, dict):
            return {str(k): self._to_jsonable(v) for k, v in value.items()}

        if isinstance(value, (list, tuple)):
            return [self._to_jsonable(v) for v in value]

        # fallback: avoid crashing json dump
        return str(value)

    def log_step(self, reward: float, info: Optional[Dict] = None):
        self.global_step += 1
        if self.enable_tensorboard:
            self.writer.add_scalar('Step/Reward', reward, self.global_step)
            if info:
                for key, value in info.items():
                    if isinstance(value, (int, float)):
                        self.writer.add_scalar(f'Step/{key}', value, self.global_step)

    def log_episode(
        self,
        episode_reward: float,
        episode_length: int,
        coverage: float,
        info: Optional[Dict] = None,
    ):
        self.episode_count += 1
        self.best_reward = max(self.best_reward, episode_reward)
        self.best_coverage = max(self.best_coverage, coverage)
        info_json = self._to_jsonable(info or {})
        episode_time_sec = float(info_json.get('episode_elapsed_seconds', 0.0))
        reset_time_sec = float(info_json.get('reset_duration_seconds', 0.0))

        if self.enable_tensorboard:
            self.writer.add_scalar('Episode/Reward', episode_reward, self.episode_count)
            self.writer.add_scalar('Episode/Length', episode_length, self.episode_count)
            self.writer.add_scalar('Episode/Coverage', coverage, self.episode_count)
            self.writer.add_scalar('Episode/DurationSec', episode_time_sec, self.episode_count)
            self.writer.add_scalar('Episode/ResetDurationSec', reset_time_sec, self.episode_count)
            self.writer.add_scalar('Episode/BestReward', self.best_reward, self.episode_count)
            self.writer.add_scalar('Episode/BestCoverage', self.best_coverage, self.episode_count)
            if info:
                for key, value in info.items():
                    if isinstance(value, (int, float)):
                        self.writer.add_scalar(f'Episode/{key}', value, self.episode_count)

        elapsed_time = self.previous_total_time_sec + (time.time() - self.start_time)
        log_msg = (
            f"Episode {self.episode_count:4d} | "
            f"Reward: {episode_reward:8.2f} | "
            f"Length: {episode_length:4d} | "
            f"Coverage: {coverage*100:5.1f}% | "
            f"EpTime: {episode_time_sec:7.2f}s | "
            f"Reset: {reset_time_sec:6.2f}s | "
            f"Wall: {elapsed_time/60:.1f}min\n"
        )
        with open(self.log_file, 'a') as f:
            f.write(log_msg)

        metrics = {
            'episode': self.episode_count,
            'reward': episode_reward,
            'length': episode_length,
            'coverage': coverage,
            'wall_time_seconds': elapsed_time,
            'episode_time_seconds': episode_time_sec,
            'reset_time_seconds': reset_time_sec,
        }
        if info_json:
            metrics.update(info_json)
        self.metrics_history.append(metrics)
        self._save_combined_data()

        if self.episode_count % self.save_interval == 0:
            self._save_metrics()

    def log_training_info(self, info: Dict[str, Any], step: Optional[int] = None):
        step = step or self.global_step
        if self.enable_tensorboard:
            for key, value in info.items():
                if isinstance(value, (int, float)):
                    self.writer.add_scalar(f'Training/{key}', value, step)

    def log_evaluation(
        self,
        episode: int,
        mean_reward: float,
        mean_length: float,
        mean_coverage: float,
        info: Optional[Dict] = None,
    ):
        self.best_eval_reward = max(self.best_eval_reward, mean_reward)
        self.best_eval_coverage = max(self.best_eval_coverage, mean_coverage)

        if self.enable_tensorboard:
            self.writer.add_scalar('Eval/MeanReward', mean_reward, episode)
            self.writer.add_scalar('Eval/MeanLength', mean_length, episode)
            self.writer.add_scalar('Eval/MeanCoverage', mean_coverage, episode)
            self.writer.add_scalar('Eval/BestReward', self.best_eval_reward, episode)
            self.writer.add_scalar('Eval/BestCoverage', self.best_eval_coverage, episode)
            if info:
                for key, value in info.items():
                    if isinstance(value, (int, float)):
                        self.writer.add_scalar(f'Eval/{key}', value, episode)

        log_msg = (
            f"Eval @ Episode {episode:4d} | "
            f"MeanReward: {mean_reward:8.2f} | "
            f"MeanLength: {mean_length:6.1f} | "
            f"MeanCoverage: {mean_coverage*100:5.1f}%\n"
        )
        with open(self.eval_log_file, 'a') as f:
            f.write(log_msg)

        metrics = {
            'episode': episode,
            'mean_reward': mean_reward,
            'mean_length': mean_length,
            'mean_coverage': mean_coverage,
        }
        if info:
            metrics.update(self._to_jsonable(info))
        self.evaluation_history.append(metrics)
        self._save_evaluations()

    def _save_metrics(self):
        with open(self.metrics_file, 'w') as f:
            json.dump(self.metrics_history, f, indent=2)
        self._save_combined_data()

    def _save_evaluations(self):
        with open(self.eval_metrics_file, 'w') as f:
            json.dump(self.evaluation_history, f, indent=2)
        self._save_combined_data()

    def _save_combined_data(self):
        payload = {
            'meta': {
                'start_time_epoch': self.start_time,
                'start_time_str': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.start_time)),
                'episode_count': self.episode_count,
                'global_step': self.global_step,
                'resume': self.resume,
            },
            'episodes': self.metrics_history,
            'evaluations': self.evaluation_history,
            'summary': {
                'best_reward': self.best_reward,
                'best_coverage': self.best_coverage,
                'best_eval_reward': self.best_eval_reward,
                'best_eval_coverage': self.best_eval_coverage,
                'total_time_sec': self.previous_total_time_sec + (time.time() - self.start_time),
            },
        }
        with open(self.combined_metrics_path, 'w') as f:
            json.dump(payload, f, indent=2)

    def save_report(self):
        report = {
            'total_episodes': self.episode_count,
            'best_reward': self.best_reward,
            'best_coverage': self.best_coverage,
            'best_eval_reward': self.best_eval_reward,
            'best_eval_coverage': self.best_eval_coverage,
            'total_time_sec': self.previous_total_time_sec + (time.time() - self.start_time),
        }
        report_path = self.log_dir / 'report.json'
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2)
        self._save_combined_data()
        print(f"[RL-MONITOR] report saved: {report_path}")

    def close(self):
        self._save_combined_data()
        if self.writer:
            self.writer.close()
        print("[RL-MONITOR] closed")

    def get_last_evaluation_episode(self) -> int:
        if not self.evaluation_history:
            return 0
        return max(
            self._episode_index_from_item(item, idx + 1)
            for idx, item in enumerate(self.evaluation_history)
        )
