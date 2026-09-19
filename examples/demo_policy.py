"""Demonstration mode: watch a policy (engineered or learned) fly the navigation task in the browser.

    .venv/bin/python examples/demo_policy.py --policy waypoint
    .venv/bin/python examples/demo_policy.py --policy experiments/<id>/model.zip --episodes 5

Runs against the dev server (port 8000) in stepped mode, paced to real time so it is watchable,
and pushes the task overlay (target ring, distance, reward, action) to the browser.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "client"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "learn"))
import numpy as np  # noqa: E402
from simclient import Simulation  # noqa: E402
from envdr3d_learn.task import NavigationTask, TaskConfig  # noqa: E402
from envdr3d_learn.policies import make_policy  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="waypoint"); ap.add_argument("--level", default="open")
    ap.add_argument("--observation", default="state"); ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--pace", type=float, default=1.0, help="real-time factor for playback (0 = as fast as possible)")
    ap.add_argument("--seed", type=int, default=7); ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=8000)
    a = ap.parse_args()
    sim = Simulation(a.host, a.port, timeout=120)
    cfg = TaskConfig(level=a.level, observation=a.observation, seed=1)
    task = NavigationTask(sim, cfg, overlay=True)
    policy = make_policy(a.policy)
    scaled = a.policy not in ("waypoint", "random")
    if scaled:
        from envdr3d_learn.train_ppo import OBS_SCALE
    step_dt = cfg.action_repeat * 0.004
    for ep in range(a.episodes):
        obs, info = task.reset(seed=a.seed + ep)
        print(f"episode {ep}: start {tuple(round(v,1) for v in task.start)} -> target {tuple(round(v,1) for v in task.target)}")
        done = False
        while not done:
            t0 = time.time()
            act = policy(obs / OBS_SCALE) if scaled else policy(obs)
            obs, r, term, trunc, info = task.step(act)
            done = term or trunc
            if a.pace > 0:
                time.sleep(max(0.0, step_dt / a.pace - (time.time() - t0)))
        res = info["result"]
        print(f"  -> {res.termination} in {res.steps} steps ({res.sim_time:.1f}s sim), final distance {res.final_distance:.2f} m")
        time.sleep(1.5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
