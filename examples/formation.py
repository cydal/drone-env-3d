"""Controller 4 — Multi-drone formation (stepped): three drones hold a triangle
while the formation centre flies a straight line. One process, many agents."""
from __future__ import annotations

from _common import connect, fmt, p_control, parser

OFFSETS = {"drone_01": (0.0, 0.0, 0.0), "drone_02": (-3.0, 3.0, -1.0), "drone_03": (-3.0, -3.0, -1.0)}


def main() -> int:
    ap = parser("formation test (stepped)")
    ap.add_argument("--alt", type=float, default=8.0); ap.add_argument("--distance", type=float, default=25.0)
    a = ap.parse_args()
    sim = connect(a)
    sim.reset(scenario=a.scenario, seed=a.seed, mode="stepped")
    agents = [x["agent_id"] for x in sim.agents() if x["agent_id"] in OFFSETS]
    # This engineering test needs positions for every drone. Agents whose observation
    # profile hides `state` (e.g. drone_02: navigation) fall back to privileged ground
    # truth, which is exactly what a policy must NOT do — hence the loud note.
    privileged = [a for a in agents if sim.observe(a).position is None]
    if privileged:
        print(f"note: using ground truth (/entities) for {privileged} — their profile hides state")

    def observe_all():
        obs = sim.observe_all()
        for a in privileged:
            st = sim.entity_state(a)
            obs[a] = type(obs[a])({**obs[a], "state": st})
        return obs

    obs = observe_all()
    centre0 = (0.0, 0.0, a.alt)
    # phase 1: climb into formation
    res = None
    for _ in range(120):
        acts = {}
        for aid in agents:
            o = obs[aid]
            tgt = tuple(c + d for c, d in zip(centre0, OFFSETS[aid]))
            vx, vy, vz, _ = p_control(o.position, tgt, speed=2.5)
            acts[aid] = sim.velocity(vx, vy, vz, frame="world")
        res = sim.step(actions=acts, steps=25)
        obs = observe_all()
        if all(p_control(obs[aid].position, tuple(c + d for c, d in zip(centre0, OFFSETS[aid])))[3] < 0.4 for aid in agents):
            break
    print("formation assembled at t=%.1fs" % res.episode.sim_time)
    # phase 2: translate the formation
    steps = int(a.distance / (1.5 * 0.1))
    worst = 0.0
    for i in range(steps):
        cx = centre0[0] + 1.5 * 0.1 * (i + 1)
        acts = {}
        for aid in agents:
            tgt = (cx + OFFSETS[aid][0], centre0[1] + OFFSETS[aid][1], centre0[2] + OFFSETS[aid][2])
            vx, vy, vz, err = p_control(obs[aid].position, tgt, gain=1.5, speed=4.0)
            acts[aid] = sim.velocity(vx + 1.5, vy, vz, frame="world")
            worst = max(worst, err)
        res = sim.step(actions=acts, steps=25)
        obs = observe_all()
    for aid in agents:
        print(f"  {aid}: {fmt(obs[aid].position)}")
    print(f"max formation error {worst:.2f} m ->", "PASS" if worst < 1.5 else "FAIL")
    return 0 if worst < 1.5 else 1


if __name__ == "__main__":
    raise SystemExit(main())
