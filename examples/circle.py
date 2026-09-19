"""Controller 3 — Circular trajectory (stepped): track a moving reference point."""
from __future__ import annotations

import math

from _common import connect, fmt, p_control, parser


def main() -> int:
    ap = parser("circle test (stepped)"); ap.add_argument("--agent", default="drone_01")
    ap.add_argument("--radius", type=float, default=6.0); ap.add_argument("--alt", type=float, default=6.0)
    ap.add_argument("--laps", type=float, default=1.0); ap.add_argument("--period", type=float, default=20.0)
    a = ap.parse_args()
    sim = connect(a)
    sim.reset(scenario=a.scenario, seed=a.seed, mode="stepped")
    obs = sim.observe(a.agent)
    cx, cy = obs.position[0], obs.position[1] - a.radius
    # climb first (server-side waypoint controller does the low-level work)
    res = sim.step(agent=a.agent, action=sim.waypoint(obs.position[0], obs.position[1], a.alt, speed=2.0), steps=1000)
    dt = 0.1                                  # 25 physics steps @ 4 ms
    n = int(a.laps * a.period / dt)
    errs = []
    t = 0.0
    for i in range(n):
        t += dt
        ang = 2 * math.pi * t / a.period + math.pi / 2
        ref = (cx + a.radius * math.cos(ang), cy + a.radius * math.sin(ang), a.alt)
        # feed-forward tangential velocity + P correction, world frame
        w = 2 * math.pi / a.period
        ffx, ffy = -a.radius * w * math.sin(ang), a.radius * w * math.cos(ang)
        obs = res.observations[a.agent]
        vx, vy, vz, dist = p_control(obs.position, ref, gain=1.5, speed=5.0)
        res = sim.step(agent=a.agent, action=sim.velocity(vx + ffx, vy + ffy, vz, frame="world"), steps=25)
        errs.append(dist)
        if i % 25 == 0:
            print(f"t={t:5.1f}s pos={fmt(obs.position)} ref={fmt(ref)} err={dist:.2f}")
    mean, worst = sum(errs) / len(errs), max(errs)
    print(f"tracking error: mean {mean:.2f} m, max {worst:.2f} m ->", "PASS" if mean < 1.0 else "FAIL")
    return 0 if mean < 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
