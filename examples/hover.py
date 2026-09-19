"""Controller 1 — Hover: take off to an altitude and hold position. Realtime mode."""
from __future__ import annotations

import time

from _common import connect, fmt, parser


def main() -> int:
    ap = parser("hover test"); ap.add_argument("--agent", default="drone_01"); ap.add_argument("--alt", type=float, default=5.0)
    ap.add_argument("--seconds", type=float, default=8.0)
    a = ap.parse_args()
    sim = connect(a)
    if not a.no_reset:
        sim.reset(scenario=a.scenario, seed=a.seed, mode="realtime")
    sim.resume()
    start = sim.entity_state(a.agent)["pose"]["position"]
    target = (start["x"], start["y"], a.alt)
    sim.act(a.agent, sim.waypoint(*target, speed=2.0, tolerance=0.2))
    print(f"[{a.agent}] climbing to {fmt(target)}")
    t0 = time.time()
    while time.time() - t0 < 10:
        p = sim.entity_state(a.agent)["pose"]["position"]
        if abs(p["z"] - a.alt) < 0.25:
            break
        time.sleep(0.2)
    print(f"[{a.agent}] holding for {a.seconds}s")
    sim.act(a.agent, sim.hold())
    worst = 0.0
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        p = sim.entity_state(a.agent)["pose"]["position"]
        drift = ((p["x"] - target[0]) ** 2 + (p["y"] - target[1]) ** 2 + (p["z"] - target[2]) ** 2) ** 0.5
        worst = max(worst, drift)
        time.sleep(0.25)
    print(f"[{a.agent}] max drift while holding: {worst:.2f} m  ->", "PASS" if worst < 0.6 else "FAIL")
    return 0 if worst < 0.6 else 1


if __name__ == "__main__":
    raise SystemExit(main())
