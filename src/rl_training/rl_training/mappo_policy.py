#!/usr/bin/env python3
from __future__ import annotations

from typing import Tuple


try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    from torch.distributions import Dirichlet
    from torch.distributions import Beta
except Exception:  # pragma: no cover
    torch = None
    nn = None
    F = None
    Dirichlet = None
    Beta = None


if nn is None:
    class MAPPOPolicy:  # type: ignore[no-redef]
        def __init__(self, *args, **kwargs) -> None:
            raise RuntimeError(
                "MAPPO requires PyTorch. Activate the environment that provides torch first."
            )
else:
    class ActorNetwork(nn.Module):
        def __init__(
            self,
            obs_dim: int,
            hidden_dim: int = 128,
            action_dim: int = 4,
            action_mode: str = 'beta',
        ) -> None:
            super().__init__()
            self.action_dim = int(action_dim)
            self.action_mode = str(action_mode)
            output_dim = int(action_dim) * 2 if self.action_mode == 'beta' else int(action_dim)
            self.net = nn.Sequential(
                nn.Linear(int(obs_dim), int(hidden_dim)),
                nn.Tanh(),
                nn.Linear(int(hidden_dim), int(hidden_dim)),
                nn.Tanh(),
                nn.Linear(int(hidden_dim), output_dim),
            )

        def forward(self, obs: torch.Tensor):
            logits = self.net(obs)
            if self.action_mode == 'dirichlet':
                alpha = F.softplus(logits) + 1e-3
                return alpha
            alpha_raw, beta_raw = torch.chunk(logits, 2, dim=-1)
            alpha = F.softplus(alpha_raw) + 1e-3
            beta = F.softplus(beta_raw) + 1e-3
            return alpha, beta


    class CriticNetwork(nn.Module):
        def __init__(self, obs_dim: int, hidden_dim: int = 128) -> None:
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(int(obs_dim), int(hidden_dim)),
                nn.Tanh(),
                nn.Linear(int(hidden_dim), int(hidden_dim)),
                nn.Tanh(),
                nn.Linear(int(hidden_dim), 1),
            )

        def forward(self, obs: torch.Tensor) -> torch.Tensor:
            return self.net(obs).squeeze(-1)


    class MAPPOPolicy(nn.Module):
        def __init__(
            self,
            local_obs_dim: int,
            shared_obs_dim: int,
            hidden_dim: int = 128,
            action_mode: str = 'beta',
        ) -> None:
            super().__init__()
            self.action_mode = str(action_mode).lower()
            if self.action_mode not in ('beta', 'dirichlet'):
                raise ValueError(f'Unsupported action_mode: {action_mode}')
            self.actor = ActorNetwork(
                obs_dim=local_obs_dim,
                hidden_dim=hidden_dim,
                action_mode=self.action_mode,
            )
            self.critic = CriticNetwork(obs_dim=shared_obs_dim, hidden_dim=hidden_dim)

        @staticmethod
        def _sanitize_actions(actions: torch.Tensor) -> torch.Tensor:
            return torch.clamp(actions, min=1e-4, max=1.0 - 1e-4)

        @staticmethod
        def _normalize_actions(actions: torch.Tensor) -> torch.Tensor:
            clipped = torch.clamp(actions, min=1e-6)
            return clipped / clipped.sum(dim=-1, keepdim=True)

        def action_distribution(self, local_obs: torch.Tensor):
            params = self.actor(local_obs)
            if self.action_mode == 'dirichlet':
                return Dirichlet(params)
            alpha, beta = params
            return Beta(alpha, beta)

        def deterministic_actions(self, local_obs: torch.Tensor) -> torch.Tensor:
            dist = self.action_distribution(local_obs)
            actions = dist.mean
            if self.action_mode == 'dirichlet':
                return self._normalize_actions(actions)
            return self._sanitize_actions(actions)

        def sample_actions(self, local_obs: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
            dist = self.action_distribution(local_obs)
            actions = dist.sample()
            if self.action_mode == 'dirichlet':
                actions = self._normalize_actions(actions)
                log_probs = dist.log_prob(actions)
                entropy = dist.entropy()
                return actions, log_probs, entropy
            actions = self._sanitize_actions(actions)
            log_probs = dist.log_prob(actions).sum(dim=-1)
            entropy = dist.entropy().sum(dim=-1)
            return actions, log_probs, entropy
