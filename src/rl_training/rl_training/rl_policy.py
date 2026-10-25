import os
import numpy as np

try:
    import torch
    import torch.nn as nn
except Exception:
    torch = None
    nn = None


class TorchPolicy:
    def __init__(
        self,
        state_dim: int = 7,
        action_dim: int = 4,
        hidden_dim: int = 64,
        device: str = 'cpu',
    ):
        if torch is None:
            raise RuntimeError('PyTorch is not available')
        self.device = torch.device(device)
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
        ).to(self.device)

    def act(self, state: np.ndarray, temperature: float = 1.0) -> np.ndarray:
        self.net.eval()
        with torch.no_grad():
            s = torch.tensor(state, dtype=torch.float32, device=self.device)
            logits = self.net(s)
            temp = max(float(temperature), 1e-6)
            weights = torch.softmax(logits / temp, dim=-1)
            return weights.cpu().numpy()

    def load(self, path: str) -> bool:
        if not os.path.exists(path):
            return False
        self.net.load_state_dict(torch.load(path, map_location=self.device))
        return True


def build_policy(
    state_dim: int,
    action_dim: int,
    hidden_dim: int,
    device: str,
):
    if torch is None:
        raise RuntimeError('PyTorch is not available; cannot build RL policy without a model.')
    return TorchPolicy(
        state_dim=state_dim,
        action_dim=action_dim,
        hidden_dim=hidden_dim,
        device=device,
    )
