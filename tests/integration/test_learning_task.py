"""Phase 3: the navigation task on a real simulator (uses the shared integration server)."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learn"))

from envdr3d_learn.task import LEVELS, NavigationTask, TaskConfig, TrajectoryRecorder  # noqa: E402
from envdr3d_learn.policies import RandomPolicy, WaypointPolicy  # noqa: E402
from envdr3d_learn.evalsets import load as load_evalset  # noqa: E402

pytestmark = pytest.mark.integration


def _run(task, policy, **reset_kw):
    obs, info = task.reset(**reset_kw)
    assert obs.shape == (12,) and np.all(np.isfinite(obs))
    done = False
    while not done:
        obs, r, term, trunc, info = task.step(policy(obs))
        done = term or trunc
    return info["result"]


def test_task_reset_randomises_and_is_reproducible(sim):
    task = NavigationTask(sim, TaskConfig(level="open", seed=1), overlay=False)
    task.reset(seed=5); a = (task.start, task.target)
    task.reset(seed=6); b = (task.start, task.target)
    task.reset(seed=5); c = (task.start, task.target)
    assert a != b and a == c                      # randomised, and reproducible from the episode seed
    s, t = LEVELS["open"]["start"], LEVELS["open"]["target"]
    assert s.x[0] <= a[0][0] <= s.x[1] and t.x[0] <= a[1][0] <= t.x[1]


def test_waypoint_baseline_reaches_target_and_random_does_not(sim):
    task = NavigationTask(sim, TaskConfig(level="open", seed=1, max_steps=120), overlay=False)
    res = _run(task, WaypointPolicy(), seed=11)
    assert res.success and res.termination == "reached" and res.final_distance < 1.0 and res.collisions == 0
    res = _run(task, RandomPolicy(0), seed=11)
    assert not res.success and res.termination == "truncated" and res.steps == 120


def test_reward_and_events_flow_from_simulator_to_task(sim):
    task = NavigationTask(sim, TaskConfig(level="open", seed=1), overlay=False)
    obs, _ = task.reset(seed=3)
    d0 = float(obs[9])
    obs, r, term, trunc, info = task.step(WaypointPolicy()(obs))
    rc = task.cfg.reward
    # progress term positive; the dense distance penalty (0.02 * ~22 m) dominates the first step by design
    assert info["distance"] < d0
    assert r > -(rc.distance_penalty * d0 + rc.time_penalty + 3 * rc.action_penalty) - 1e-6
    assert info["events"] == [] and not term and not trunc
    # reached event comes from the task (distance), termination flags follow
    res = _run(task, WaypointPolicy(), seed=3)
    assert res.success and res.return_ > 5.0


def test_fixed_eval_pairs_and_navigation_observation_mode(sim):
    es = load_evalset("level1_open_eval")
    pair = es["pairs"][0]
    for mode in ("state", "navigation"):
        task = NavigationTask(sim, TaskConfig(level="open", observation=mode, seed=1, max_steps=150), overlay=False)
        res = _run(task, WaypointPolicy(), seed=0, pair=pair)
        assert tuple(res.start) == tuple(pair["start"]) and tuple(res.target) == tuple(pair["target"])
        assert res.success, f"waypoint failed under {mode}: {res}"


def test_trajectory_recorder_writes_aligned_dataset(sim, tmp_path):
    rec = TrajectoryRecorder(tmp_path, "ds", meta={"policy": "waypoint"})
    task = NavigationTask(sim, TaskConfig(level="open", seed=1, max_steps=150), recorder=rec, overlay=False)
    res = _run(task, WaypointPolicy(), seed=8)
    files = sorted((tmp_path / "ds").glob("episode_*.npz"))
    assert len(files) == 1
    d = np.load(files[0], allow_pickle=False)
    T = d["action"].shape[0]
    assert d["obs"].shape == (T + 1, 12) and d["reward"].shape == (T,) and d["position"].shape == (T + 1, 3)
    assert T == res.steps
    idx = [json.loads(l) for l in (tmp_path / "ds" / "index.jsonl").open()]
    assert idx[0]["success"] is True and idx[0]["file"] == files[0].name


def test_saturated_actions_are_clamped_not_rejected(sim):
    """Regression: a full diagonal action exceeded the agent's speed limit, the simulator rejected
    it and the drone silently never moved (6/20 'failures' of the first learned policy)."""
    task = NavigationTask(sim, TaskConfig(level="open", seed=1, max_steps=30), overlay=False)
    obs, _ = task.reset(seed=21)
    p0 = np.array(task.prev_pos)
    for _ in range(20):                                                                 # 2 s of sim
        obs, r, term, trunc, info = task.step(np.array([1.0, 1.0, 0.0], np.float32))   # 4.24 m/s requested
        if term or trunc:
            break
    moved = np.linalg.norm(np.array(info["position"]) - p0)
    # the controller's 2 m/s^2 acceleration limit gives ~3 m in 2 s; the bug gave exactly 0.00
    assert moved > 1.5, f"drone did not move after saturated actions ({moved:.2f} m)"
