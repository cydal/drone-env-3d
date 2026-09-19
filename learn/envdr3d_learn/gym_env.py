"""Gymnasium adapter over NavigationTask, so standard RL libraries can train against the
simulator without knowing anything about it. Also: a launcher for N parallel simulators
(each its own API process on its own port + gz-transport partition)."""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import gymnasium as gym
import numpy as np

from simclient import Simulation

from .task import NavigationTask, TaskConfig

ROOT = Path(__file__).resolve().parents[2]


class NavigationGymEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, cfg: TaskConfig, host: str = "127.0.0.1", port: int = 8000, overlay: bool = False,
                 recorder=None) -> None:
        super().__init__()
        self.sim = Simulation(host, port, timeout=120)
        self.task = NavigationTask(self.sim, cfg, recorder=recorder, overlay=overlay)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(self.task.obs_dim,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(self.task.act_dim,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        obs, info = self.task.reset(seed=seed, pair=(options or {}).get("pair"))
        return obs, _plain(info)

    def step(self, action):
        obs, r, term, trunc, info = self.task.step(action)
        return obs, r, term, trunc, _plain(info)

    def close(self):
        self.sim.close()


def _plain(info: dict) -> dict:
    out = {}
    for k, v in info.items():
        if k == "result":
            out["result"] = v.__dict__
        elif isinstance(v, (int, float, str, bool, list, tuple)) or v is None:
            out[k] = v
    return out


class SimulatorPool:
    """Launch N Simulation API servers (each with its own headless Gazebo) for parallel envs."""

    def __init__(self, n: int, base_port: int = 8101, partition_prefix: str = "envdr3d-train") -> None:
        self.n, self.base_port = n, base_port
        self.procs: list[subprocess.Popen] = []
        self.ports = [base_port + i for i in range(n)]
        for i, port in enumerate(self.ports):
            env = os.environ.copy()
            env.update({"GZ_PARTITION": f"{partition_prefix}-{i}", "SIMAPI_PORT": str(port), "PYTHONPATH": str(ROOT / "api"),
                        "SIMAPI_TELEMETRY_HZ": "5"})
            log = open(ROOT / "runs" / f"train-sim-{i}.log", "ab")
            self.procs.append(subprocess.Popen([sys.executable, "-m", "simapi"], env=env, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        self._wait_ready()

    def _wait_ready(self, timeout: float = 60.0) -> None:
        import httpx
        deadline = time.time() + timeout
        for port in self.ports:
            while time.time() < deadline:
                try:
                    if httpx.get(f"http://127.0.0.1:{port}/status", timeout=2).status_code == 200:
                        break
                except httpx.HTTPError:
                    time.sleep(0.3)
            else:
                raise TimeoutError(f"simulator API on port {port} did not start")

    def close(self) -> None:
        for p in self.procs:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for p in self.procs:
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
