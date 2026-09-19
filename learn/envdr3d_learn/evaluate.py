"""Evaluate any policy on a fixed evaluation set and record metrics + per-episode trajectories.

    python -m envdr3d_learn.evaluate --policy waypoint --evalset level1_open_eval
    python -m envdr3d_learn.evaluate --policy experiments/<id>/model.zip --evalset level1_open_eval --record
"""
from __future__ import annotations

import argparse
import json
import statistics as stats
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from simclient import Simulation

from .evalsets import load as load_evalset
from .experiment import EXPERIMENTS, ROOT
from .policies import make_policy
from .task import EpisodeResult, NavigationTask, TaskConfig, TrajectoryRecorder


def summarize(results: list[EpisodeResult]) -> dict[str, Any]:
    n = len(results)
    succ = [r for r in results if r.success]
    def mean(xs): return float(stats.fmean(xs)) if xs else None
    return {
        "episodes": n,
        "success_rate": len(succ) / n if n else 0.0,
        "collision_rate": sum(r.termination == "collision" for r in results) / n if n else 0.0,
        "timeout_rate": sum(r.termination in ("timeout", "truncated") for r in results) / n if n else 0.0,
        "out_of_bounds_rate": sum(r.termination == "out_of_bounds" for r in results) / n if n else 0.0,
        "mean_time_to_target_s": mean([r.sim_time for r in succ]),
        "mean_path_efficiency": mean([min(1.0, r.path_efficiency) for r in succ]),
        "mean_final_distance_m": mean([r.final_distance for r in results]),
        "mean_return": mean([r.return_ for r in results]),
        "mean_action_delta": mean([r.mean_abs_action_delta for r in results]),
        "terminations": {t: sum(r.termination == t for r in results) for t in sorted({r.termination for r in results})},
    }


def run_eval(policy, cfg: TaskConfig, evalset: dict[str, Any], *, sim: Simulation, out_dir: Path | None = None,
             record: bool = False, overlay: bool = False, verbose: bool = True) -> dict[str, Any]:
    recorder = TrajectoryRecorder(out_dir, "trajectories", meta={"policy": policy.name, "evalset": evalset.get("name")}) \
        if (record and out_dir) else None
    task = NavigationTask(sim, cfg, recorder=recorder, overlay=overlay)
    results: list[EpisodeResult] = []
    per_episode = []
    t_start = time.time()
    from simclient.simulation import SimulationError
    for i, pair in enumerate(evalset["pairs"]):
        for attempt in range(3):
            try:
                policy.reset()
                obs, info = task.reset(seed=1000 + i, pair=pair)
                done = False
                trace = []
                while not done:
                    a = policy(obs)
                    obs, r, term, trunc, info = task.step(a)
                    done = term or trunc
                    trace.append({"t": round(info["sim_time"], 3), "distance": round(info["distance"], 3),
                                  "position": [round(v, 3) for v in info["position"]], "velocity": [round(v, 3) for v in info["velocity"]],
                                  "action": [round(float(v), 3) for v in a], "reward": round(float(r), 4), "events": info["events"]})
                break
            except (SimulationError, RuntimeError) as e:
                # a wedged/crashed simulator must not invalidate an evaluation: relaunch and redo the pair
                print(f"  [{pair['id']}] simulator error ({str(e)[:80]}); relaunching (attempt {attempt + 1})")
                try:
                    sim.shutdown()
                except Exception:
                    pass
                task._loaded = False
                time.sleep(1.0)
        else:
            raise RuntimeError(f"evaluation of {pair['id']} failed after 3 simulator relaunches")
        res: EpisodeResult = info["result"]
        results.append(res)
        per_episode.append({"pair": pair["id"], **res.__dict__, "trace": trace})
        if verbose:
            print(f"  [{pair['id']}] {res.termination:12s} steps={res.steps:3d} t={res.sim_time:5.1f}s "
                  f"final={res.final_distance:5.2f}m eff={min(1.0, res.path_efficiency):.2f} return={res.return_:6.1f}")
    summary = summarize(results)
    summary["wall_time_s"] = round(time.time() - t_start, 1)
    summary["policy"] = policy.name
    summary["evalset"] = evalset.get("name")
    summary["task_config"] = cfg.to_json()
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=str))
        with (out_dir / "episodes.jsonl").open("w") as f:
            for e in per_episode:
                f.write(json.dumps(e, default=str) + "\n")
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description="evaluate a policy on a fixed evaluation set")
    ap.add_argument("--policy", required=True, help="random | waypoint | path/to/model.zip")
    ap.add_argument("--evalset", default="level1_open_eval")
    ap.add_argument("--observation", default="state", choices=["state", "navigation", "vision"])
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--out", default=None, help="output dir (default experiments/eval_<policy>_<evalset>_<stamp>)")
    ap.add_argument("--record", action="store_true", help="also save .npz trajectories")
    ap.add_argument("--overlay", action="store_true", help="push task overlay to the browser")
    ap.add_argument("--max-steps", type=int, default=200)
    a = ap.parse_args()
    es = load_evalset(a.evalset); es["name"] = a.evalset
    cfg = TaskConfig(level=es["level"], observation=a.observation, max_steps=a.max_steps, seed=1)
    policy = make_policy(a.policy)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    out = Path(a.out) if a.out else EXPERIMENTS / f"eval_{policy.name}_{a.evalset}_{stamp}"
    sim = Simulation(a.host, a.port, timeout=120)
    print(f"evaluating {policy.name} on {a.evalset} ({es['n']} episodes, observation={a.observation})")
    s = run_eval(policy, cfg, es, sim=sim, out_dir=out, record=a.record, overlay=a.overlay)
    print(json.dumps({k: v for k, v in s.items() if k not in ("task_config",)}, indent=2))
    print("written:", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
