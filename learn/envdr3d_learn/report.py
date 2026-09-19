"""Summarise an experiment directory as a markdown table (final eval vs baselines, eval history).

    python -m envdr3d_learn.report experiments/<id>
"""
from __future__ import annotations

import sys

from .experiment import load_experiment


def fmt(v, nd=2):
    return "—" if v is None else (f"{v:.{nd}f}" if isinstance(v, float) else str(v))


def main() -> int:
    meta = load_experiment(sys.argv[1])
    res = meta.get("results", {})
    cfg = meta["config"]
    print(f"**Experiment** `{meta['experiment_id']}` — {cfg.get('algorithm')} · observation `{cfg.get('observation_profile')}` · "
          f"level `{cfg['task']['level']}` · {cfg.get('total_steps')} steps · {cfg.get('n_envs')} simulators · seed {cfg.get('seed')} · commit {meta.get('git_commit')}")
    if res.get("train_wall_time_s"):
        print(f"Training wall time {res['train_wall_time_s']} s ({res.get('steps_per_second')} steps/s).\n")
    rows = [("PPO (frozen, deterministic)", res.get("final_eval")), ("waypoint baseline", res.get("baseline_waypoint")),
            ("random baseline", res.get("baseline_random"))]
    print("| policy | success | collision | timeout | time to target (s) | path eff. | final dist (m) | action Δ |")
    print("|---|---|---|---|---|---|---|---|")
    for name, r in rows:
        if not r:
            continue
        print(f"| {name} | {fmt(r['success_rate'])} | {fmt(r['collision_rate'])} | {fmt(r['timeout_rate'])} | "
              f"{fmt(r['mean_time_to_target_s'], 1)} | {fmt(r['mean_path_efficiency'])} | {fmt(r['mean_final_distance_m'])} | {fmt(r['mean_action_delta'], 3)} |")
    hist = res.get("eval_history") or []
    if hist:
        print("\n| training steps | success | final dist (m) | mean return |\n|---|---|---|---|")
        for h in hist:
            print(f"| {h['timesteps']} | {fmt(h['success_rate'])} | {fmt(h['mean_final_distance_m'])} | {fmt(h['mean_return'], 1)} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
