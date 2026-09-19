"""Policies are external components: obs -> action. The simulator never imports them.

    policy = WaypointPolicy()           # engineered baseline
    policy = RandomPolicy(seed=0)       # diagnostic floor
    policy = SB3Policy.load(path)       # learned (frozen) PPO policy
    action = policy(obs)
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Protocol

import numpy as np


class Policy(Protocol):
    name: str
    def __call__(self, obs: np.ndarray) -> np.ndarray: ...
    def reset(self) -> None: ...


class RandomPolicy:
    name = "random"

    def __init__(self, seed: int = 0, act_dim: int = 3) -> None:
        self.rng = np.random.default_rng(seed)
        self.act_dim = act_dim

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        return self.rng.uniform(-1, 1, self.act_dim).astype(np.float32)

    def reset(self) -> None:
        pass


class WaypointPolicy:
    """Engineered baseline: proportional velocity toward the target, in the same action space
    the learned policy uses (normalised world-frame velocity). Uses only the task observation
    (relative target vector), so it is a fair comparison under any observation mode."""
    name = "waypoint"

    def __init__(self, gain: float = 0.6, max_norm: float = 1.0) -> None:
        self.gain, self.max_norm = gain, max_norm

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        rel = obs[:3]
        a = self.gain * rel
        n = float(np.linalg.norm(a))
        if n > self.max_norm:
            a = a * self.max_norm / n
        return np.clip(a, -1, 1).astype(np.float32)

    def reset(self) -> None:
        pass


class SB3Policy:
    """Frozen stable-baselines3 policy (deterministic actions). Loaded lazily so the rest of
    the package works without torch installed. Observation scaling used at training time is
    read from the experiment.json next to (or above) the model file, so the same policy object
    works for evaluation, demos and datasets without callers knowing about it."""
    name = "ppo"

    def __init__(self, model, deterministic: bool = True, name: str = "ppo", obs_scale=None) -> None:
        self.model, self.deterministic, self.name = model, deterministic, name
        self.obs_scale = None if obs_scale is None else np.asarray(obs_scale, np.float32)

    @classmethod
    def load(cls, path: str | Path, deterministic: bool = True) -> "SB3Policy":
        import json
        from stable_baselines3 import PPO
        path = Path(path)
        scale = None
        for parent in (path.parent, path.parent.parent):
            meta = parent / "experiment.json"
            if meta.exists():
                scale = json.loads(meta.read_text()).get("config", {}).get("obs_scale")
                break
        return cls(PPO.load(str(path), device="cpu"), deterministic, name=path.stem, obs_scale=scale)

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        x = obs / self.obs_scale if self.obs_scale is not None else obs
        action, _ = self.model.predict(x, deterministic=self.deterministic)
        return np.asarray(action, np.float32)

    def reset(self) -> None:
        pass


def make_policy(spec: str, **kw: Any) -> Policy:
    """'random' | 'waypoint' | path/to/model.zip"""
    if spec == "random":
        return RandomPolicy(**kw)
    if spec == "waypoint":
        return WaypointPolicy(**kw)
    return SB3Policy.load(spec, **kw)
