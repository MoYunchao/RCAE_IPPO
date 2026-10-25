import numpy as np


def build_observation(global_state) -> np.ndarray:
    """Normalize and pack the 7-dim global_state."""
    if global_state is None or len(global_state) < 7:
        return np.zeros((7,), dtype=np.float32)
    obs = np.array(global_state[:7], dtype=np.float32)
    obs = np.clip(obs, 0.0, 1.0)
    return obs
