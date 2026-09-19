"""Failure analysis: print what a policy saw and did, step by step.

    python -m envdr3d_learn.inspect_episode experiments/<eval_dir>/episodes.jsonl --failed
    python -m envdr3d_learn.inspect_episode datasets/<name>/episode_00003.npz
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def show_trace(title: str, rows, every: int = 1) -> None:
    print(f"\n== {title}")
    print(f"{'t':>6} {'dist':>6} {'pos x':>7} {'y':>7} {'z':>6} {'vel':>5} {'action':>20} {'reward':>7}  events")
    for i, r in enumerate(rows):
        if i % every and i != len(rows) - 1:
            continue
        v = np.linalg.norm(r["velocity"])
        print(f"{r['t']:6.2f} {r['distance']:6.2f} {r['position'][0]:7.2f} {r['position'][1]:7.2f} {r['position'][2]:6.2f} "
              f"{v:5.2f} {str([round(x, 2) for x in r['action']]):>20} {r['reward']:7.3f}  {','.join(r['events']) if r['events'] else ''}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path"); ap.add_argument("--failed", action="store_true", help="only failed episodes")
    ap.add_argument("--every", type=int, default=5, help="print every Nth step"); ap.add_argument("--limit", type=int, default=3)
    a = ap.parse_args()
    p = Path(a.path)
    if p.suffix == ".npz":
        d = np.load(p, allow_pickle=False)
        res = json.loads(str(d["result"])); events = json.loads(str(d["events"]))
        target = d["target"]
        rows = []
        for t in range(len(d["action"])):
            pos = d["position"][t + 1]
            rows.append({"t": float(d["sim_time"][t + 1]), "distance": float(np.linalg.norm(target - pos)),
                         "position": pos.tolist(), "velocity": d["velocity"][t + 1].tolist(), "action": d["action"][t].tolist(),
                         "reward": float(d["reward"][t]), "events": events[t]})
        show_trace(f"{p.name}: {res['termination']} (success={res['success']}) start={res['start']} target={res['target']}", rows, a.every)
        return 0
    shown = 0
    for line in p.open():
        e = json.loads(line)
        if a.failed and e["success"]:
            continue
        show_trace(f"{e['pair']}: {e['termination']} steps={e['steps']} final={e['final_distance']:.2f}m start={e['start']} target={e['target']}",
                   e["trace"], a.every)
        shown += 1
        if shown >= a.limit:
            break
    if shown == 0:
        print("no matching episodes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
