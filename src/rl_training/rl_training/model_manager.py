#!/usr/bin/env python3
"""
模型管理器：检查点 + 最佳模型追踪
"""

import json
import re
import time
from pathlib import Path
from typing import Optional, Dict, List


class ModelManager:
    def __init__(
        self,
        save_dir: str = 'models',
        keep_best_n: int = 3,
        save_interval: int = 10,
    ):
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)

        self.keep_best_n = keep_best_n
        self.save_interval = save_interval

        self.best_models: List[Dict] = []
        self.best_reward = float('-inf')
        self.best_coverage = 0.0
        self.best_eval_reward = float('-inf')
        self.best_eval_coverage = 0.0
        self.checkpoint_count = 0

        self.metadata_file = self.save_dir / 'models_metadata.json'
        self.latest_model_path = self.save_dir / 'latest_model.zip'
        self.latest_metadata_file = self.save_dir / 'latest_model.json'
        self.resume_state_file = self.save_dir / 'resume_state.json'
        self._load_metadata()

    def _to_jsonable(self, value):
        if value is None or isinstance(value, (bool, int, float, str)):
            return value
        if hasattr(value, 'item'):
            try:
                return value.item()
            except Exception:
                pass
        if hasattr(value, 'tolist'):
            try:
                return value.tolist()
            except Exception:
                pass
        if isinstance(value, dict):
            return {str(k): self._to_jsonable(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._to_jsonable(v) for v in value]
        return str(value)

    def _write_json(self, path: Path, data: Dict):
        with open(path, 'w') as f:
            json.dump(self._to_jsonable(data), f, indent=2)

    def _load_metadata(self):
        if self.metadata_file.exists():
            with open(self.metadata_file, 'r') as f:
                data = json.load(f)
            self.best_models = data.get('best_models', [])
            self.best_reward = data.get('best_reward', float('-inf'))
            self.best_coverage = data.get('best_coverage', 0.0)
            self.best_eval_reward = data.get('best_eval_reward', float('-inf'))
            self.best_eval_coverage = data.get('best_eval_coverage', 0.0)
            self.checkpoint_count = data.get('checkpoint_count', 0)

    def _save_metadata(self):
        data = {
            'best_models': self.best_models,
            'best_reward': self.best_reward,
            'best_coverage': self.best_coverage,
            'best_eval_reward': self.best_eval_reward,
            'best_eval_coverage': self.best_eval_coverage,
            'checkpoint_count': self.checkpoint_count,
            'last_update': time.strftime('%Y-%m-%d %H:%M:%S'),
        }
        self._write_json(self.metadata_file, data)

    def should_save_checkpoint(self, episode: int) -> bool:
        return episode % self.save_interval == 0

    @staticmethod
    def should_save_latest_model(episode: int, interval: int) -> bool:
        interval = max(1, int(interval))
        return episode % interval == 0

    @staticmethod
    def _metric_value(metadata: Dict, metric: str) -> float:
        if metric in ('reward', 'eval_reward'):
            return float(metadata.get('reward', float('-inf')))
        if metric in ('coverage', 'eval_coverage'):
            return float(metadata.get('coverage', 0.0))
        raise ValueError(f"未知的指标: {metric}")

    @staticmethod
    def _parse_episode_from_checkpoint_name(path: Path) -> int:
        # 支持 checkpoint_ep14.zip / checkpoint_ep14_r-5.zip 等历史命名
        match = re.search(r'checkpoint_ep(\d+)', path.name)
        if match:
            try:
                return int(match.group(1))
            except Exception:
                return 0
        return 0

    def save_checkpoint(
        self,
        model,
        episode: int,
        reward: float,
        coverage: float,
        num_timesteps: Optional[int] = None,
        info: Optional[Dict] = None,
    ) -> str:
        self.checkpoint_count += 1
        checkpoint_name = f'checkpoint_ep{episode}.zip'
        checkpoint_path = self.save_dir / checkpoint_name

        model.save(str(checkpoint_path))

        metadata = {
            'episode': episode,
            'reward': reward,
            'coverage': coverage,
            'num_timesteps': int(num_timesteps or 0),
            'checkpoint_count': self.checkpoint_count,
            'timestamp': time.time(),
            'time_str': time.strftime('%Y-%m-%d %H:%M:%S'),
        }
        if info:
            metadata.update(info)
        self._write_json(checkpoint_path.with_suffix('.json'), metadata)
        self._save_metadata()
        print(
            f"[RL-MODEL] checkpoint saved: episode={episode}, reward={reward:.5f}, "
            f"coverage={coverage:.4f}, path={checkpoint_path}"
        )
        return str(checkpoint_path)

    def save_latest_model(
        self,
        model,
        episode: int,
        reward: float,
        coverage: float,
        num_timesteps: int,
        info: Optional[Dict] = None,
        source_path: Optional[str] = None,
    ) -> str:
        model.save(str(self.latest_model_path))

        metadata = {
            'episode': int(episode),
            'reward': float(reward),
            'coverage': float(coverage),
            'num_timesteps': int(num_timesteps),
            'timestamp': time.time(),
            'time_str': time.strftime('%Y-%m-%d %H:%M:%S'),
            'path': str(self.latest_model_path),
            'source_path': source_path or str(self.latest_model_path),
        }
        if info:
            metadata.update(info)

        self._write_json(self.latest_metadata_file, metadata)
        self._write_json(self.resume_state_file, {
            'latest_model_path': str(self.latest_model_path),
            'latest_episode': int(episode),
            'latest_reward': float(reward),
            'latest_coverage': float(coverage),
            'latest_num_timesteps': int(num_timesteps),
            'updated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
        })
        self._save_metadata()
        print(
            f"[RL-MODEL] latest model updated: episode={episode}, timesteps={num_timesteps}, "
            f"path={self.latest_model_path}"
        )
        return str(self.latest_model_path)

    def load_resume_state(self) -> Dict:
        if self.resume_state_file.exists():
            with open(self.resume_state_file, 'r') as f:
                return json.load(f)
        return {}

    def list_checkpoints(self) -> List[Dict]:
        checkpoints = []
        for checkpoint_file in self.save_dir.glob('checkpoint_*.zip'):
            metadata_file = checkpoint_file.with_suffix('.json')
            metadata = {
                'path': str(checkpoint_file),
                'timestamp': checkpoint_file.stat().st_mtime,
                'time_str': time.strftime(
                    '%Y-%m-%d %H:%M:%S',
                    time.localtime(checkpoint_file.stat().st_mtime),
                ),
                'episode': self._parse_episode_from_checkpoint_name(checkpoint_file),
            }
            if metadata_file.exists():
                try:
                    with open(metadata_file, 'r') as f:
                        metadata.update(json.load(f))
                except Exception:
                    pass
            checkpoints.append(metadata)

        checkpoints.sort(
            key=lambda item: (
                int(item.get('episode', 0)),
                float(item.get('timestamp', 0.0)),
            )
        )
        return checkpoints

    def get_latest_resume_path(self) -> Optional[str]:
        resume_state = self.load_resume_state()
        latest_path = resume_state.get('latest_model_path', '')
        if latest_path and Path(latest_path).exists():
            return str(Path(latest_path))
        if self.latest_model_path.exists():
            return str(self.latest_model_path)
        checkpoints = self.list_checkpoints()
        if checkpoints:
            return str(checkpoints[-1].get('path'))
        return None

    def get_latest_resume_info(self) -> Dict:
        if self.latest_metadata_file.exists():
            with open(self.latest_metadata_file, 'r') as f:
                return json.load(f)
        resume_state = self.load_resume_state()
        checkpoints = self.list_checkpoints()
        if checkpoints:
            latest = dict(checkpoints[-1])
            latest.update(resume_state)
            return latest
        return resume_state

    def save_best_model(
        self,
        model,
        episode: int,
        reward: float,
        coverage: float,
        metric: str = 'reward',
        info: Optional[Dict] = None,
    ) -> bool:
        if metric == 'reward':
            is_best = reward > self.best_reward
            if is_best:
                self.best_reward = reward
        elif metric == 'coverage':
            is_best = coverage > self.best_coverage
            if is_best:
                self.best_coverage = coverage
        elif metric == 'eval_reward':
            is_best = reward > self.best_eval_reward
            if is_best:
                self.best_eval_reward = reward
        elif metric == 'eval_coverage':
            is_best = coverage > self.best_eval_coverage
            if is_best:
                self.best_eval_coverage = coverage
        else:
            raise ValueError(f"未知的指标: {metric}")

        if not is_best:
            return False

        best_model_name = f'best_model_{metric}.zip'
        best_model_path = self.save_dir / best_model_name
        model.save(str(best_model_path))

        metadata = {
            'episode': episode,
            'reward': reward,
            'coverage': coverage,
            'metric': metric,
            'timestamp': time.time(),
            'time_str': time.strftime('%Y-%m-%d %H:%M:%S'),
        }
        if info:
            metadata.update(info)
        self.best_models.append(metadata)
        same_metric = [item for item in self.best_models if item.get('metric') == metric]
        other_metrics = [item for item in self.best_models if item.get('metric') != metric]
        if len(same_metric) > self.keep_best_n:
            same_metric = sorted(
                same_metric,
                key=lambda x: self._metric_value(x, metric),
                reverse=True,
            )[:self.keep_best_n]
        self.best_models = other_metrics + same_metric
        self._save_metadata()
        print(
            f"[RL-MODEL] new best({metric}): episode={episode}, reward={reward:.5f}, "
            f"coverage={coverage:.4f}, path={best_model_path}"
        )
        return True

    def print_summary(self):
        print("\n" + "=" * 50)
        print("模型管理摘要")
        print("=" * 50)
        print(f"最佳奖励: {self.best_reward:.2f}")
        print(f"最佳覆盖率: {self.best_coverage*100:.1f}%")
        print(f"最佳评估奖励: {self.best_eval_reward:.2f}")
        print(f"最佳评估覆盖率: {self.best_eval_coverage*100:.1f}%")
        print(f"检查点数量: {self.checkpoint_count}")
        latest_info = self.get_latest_resume_info()
        if latest_info:
            print(
                f"最新续训点: episode={int(latest_info.get('episode', latest_info.get('latest_episode', 0)))} "
                f"timesteps={int(latest_info.get('num_timesteps', latest_info.get('latest_num_timesteps', 0)))}"
            )
        print("=" * 50)
