"""Dataset generation: run a policy for N randomised episodes and store trajectories.

    python -m envdr3d_learn.record --policy waypoint --level open --episodes 50 --name demos_open_waypoint
Produces datasets/<name>/episode_XXXXX.npz + index.jsonl + meta.json (see TrajectoryRecorder).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from simclient import Simulation

from .experiment import ROOT
from .policies import make_policy
from .task import NavigationTask, TaskConfig, TrajectoryRecorder


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="waypoint"); ap.add_argument("--level", default="open")
    ap.add_argument("--observation", default="state"); ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--name", required=True); ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--only-success", action="store_true", help="keep only successful episodes")
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=8000)
    a = ap.parse_args()
    sim = Simulation(a.host, a.port, timeout=120)
    cfg = TaskConfig(level=a.level, observation=a.observation, seed=1)
    rec = TrajectoryRecorder(ROOT / "datasets", a.name, meta={"policy": a.policy, "level": a.level, "observation": a.observation,
                                                              "seed": a.seed, "task_config": cfg.to_json()})
    task = NavigationTask(sim, cfg, recorder=rec, overlay=False)
    policy = make_policy(a.policy)
    kept = 0
    obs, _ = task.reset(seed=a.seed)
    for ep in range(a.episodes):
        if ep > 0:
            obs, _ = task.reset()
        policy.reset(); done = False
        while not done:
            obs, r, term, trunc, info = task.step(policy(obs)); done = term or trunc
        res = info["result"]
        if a.only_success and not res.success:
            # drop the file that was just written
            last = json.loads(rec.index.read_text().splitlines()[-1])
            (rec.dir / last["file"]).unlink(missing_ok=True)
            lines = rec.index.read_text().splitlines()[:-1]
            rec.index.write_text("\n".join(lines) + ("\n" if lines else "")); rec.n -= 1
        else:
            kept += 1
        print(f"  ep {ep:3d}: {res.termination:10s} steps={res.steps} return={res.return_:.1f}")
    print(f"dataset {rec.dir}: {kept} episodes kept")
    return 0


if __name__ == "__main__":
    sys.exit(main())
