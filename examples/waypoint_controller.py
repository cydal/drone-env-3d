"""Controller 2 — Waypoint (stepped mode): take off -> waypoint -> hover -> land.

The controller is *external*: it reads observations, computes body-frame
velocities itself and advances the simulation step by step. Nothing here knows
about Gazebo.
"""
from __future__ import annotations

import math

from _common import connect, fmt, p_control, parser


def body_frame(vx, vy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


def fly_to(sim, agent, target, *, speed=2.0, tol=0.35, max_steps=400, steps_per_action=25):
    obs = sim.observe(agent)
    for _ in range(max_steps):
        vx, vy, vz, dist = p_control(obs.position, target, speed=speed)
        if dist < tol:
            return True, obs
        bx, by = body_frame(vx, vy, obs.yaw)
        res = sim.step(agent=agent, action=sim.velocity(bx, by, vz), steps=steps_per_action)
        obs = res.observations[agent]
    return False, obs


def main() -> int:
    ap = parser("waypoint test (stepped)"); ap.add_argument("--agent", default="drone_01")
    ap.add_argument("--waypoint", default="8,4,6")
    a = ap.parse_args()
    sim = connect(a)
    ep = sim.reset(scenario=a.scenario, seed=a.seed, mode="stepped") if not a.no_reset else sim.set_mode("stepped") and sim.episode()
    print("episode", ep.episode_id, "mode", ep.mode)
    obs = sim.observe(a.agent)
    if obs.position is None:
        print("agent profile does not expose state; this controller needs the 'state' profile"); return 2
    start = obs.position
    wp = tuple(float(v) for v in a.waypoint.split(","))
    print(f"[{a.agent}] start {fmt(start)}")
    for label, tgt, spd in (("take off", (start[0], start[1], 4.0), 2.0), ("to waypoint", wp, 2.5)):
        ok, obs = fly_to(sim, a.agent, tgt, speed=spd)
        print(f"[{a.agent}] {label} -> {fmt(tgt)}: {'reached' if ok else 'TIMEOUT'} at {fmt(obs.position)} (t={obs.sim_time:.2f}s)")
        if not ok:
            return 1
    for _ in range(12):
        res = sim.step(agent=a.agent, action=sim.hold(), steps=25)
    print(f"[{a.agent}] hovered 1.2 s at {fmt(res.observations[a.agent].position)}")
    ok, obs = fly_to(sim, a.agent, (wp[0], wp[1], 0.6), speed=1.0, tol=0.25)
    ok, obs = fly_to(sim, a.agent, (wp[0], wp[1], 0.12), speed=0.5, tol=0.12, max_steps=200)
    res = sim.step(agent=a.agent, action=sim.arm(False), steps=125)
    final = res.observations[a.agent].position
    landed = [e for e in sim.events() if e.event == "landing"]
    ok = math.hypot(final[0] - wp[0], final[1] - wp[1]) < 1.0 and final[2] < 0.5
    print(f"[{a.agent}] landed at {fmt(final)}; landing events: {len(landed)}; steps={res.episode.step_count}")
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
