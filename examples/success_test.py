"""Brief 2 §29 success test, end to end, printed as a checklist.

Run the API (scripts/dev.sh) and optionally open the browser, then:
    .venv/bin/python examples/success_test.py
"""
from __future__ import annotations

import time

from _common import connect, fmt, parser


def check(ok: bool, label: str) -> bool:
    print(("  ✅ " if ok else "  ❌ ") + label)
    return ok


def main() -> int:
    a = parser("phase 2 success test").parse_args()
    sim = connect(a)
    results = []
    print("1-3. world loads, three drones spawn")
    ep = sim.reset(scenario="phase2_city", seed=18372, mode="realtime")
    agents = [x["agent_id"] for x in sim.agents()]
    results.append(check(len(agents) >= 3, f"agents: {agents}"))
    print("4-5. controller connects, receives observations")
    obs = sim.observe_all()
    results.append(check(set(obs) == set(agents) and obs["drone_02"].get("gps") is not None, "observations per profile"))
    print("6-8. actions -> drones move (browser shows it live)")
    sim.resume()
    sim.act("drone_01", sim.waypoint(0, 1.5, 6, speed=2))
    sim.act("drone_02", sim.waypoint(0, -1.5, 5, speed=2))
    sim.act("drone_03", sim.velocity(0, 0, 1.0))
    time.sleep(4)
    z = {aid: sim.entity_state(aid)["pose"]["position"]["z"] for aid in agents}
    results.append(check(all(v > 2.0 for v in z.values()), f"all airborne: { {k: round(v, 1) for k, v in z.items()} }"))
    print("9-10. one drone collides with an obstacle -> collision event")
    sim.act("drone_01", sim.waypoint(20, 10, 6, speed=5))     # into tower_01
    hit = None
    for _ in range(80):
        hit = next((e for e in sim.events() if e.event == "collision"), None)
        if hit:
            break
        time.sleep(0.25)
    results.append(check(hit is not None, f"collision event: {hit.entities if hit else None} at {fmt(hit.data['position'].values()) if hit else '-'}"))
    print("11. pause")
    st = sim.pause()
    results.append(check(st["paused"], "paused"))
    print("12. manual stepping")
    it0 = st["iterations"]
    res = sim.step(steps=25)
    results.append(check(res.status["iterations"] == it0 + 25, f"stepped 25 iterations ({it0} -> {res.status['iterations']})"))
    print("13-14. reset with same seed reproduces the same initial state")
    def probe(seed):
        sim.reset(scenario="phase2_city", seed=seed, mode="stepped")
        sim.step(actions={aid: sim.velocity(0.3, 0.1, 1.0) for aid in agents}, steps=1)
        r = sim.step(steps=300)
        return {aid: tuple(round(v, 6) for v in r.observations[aid].position) if r.observations[aid].position
                else tuple(round(sim.entity_state(aid)["pose"]["position"][k], 6) for k in "xyz") for aid in agents}
    p1, p2 = probe(18372), probe(18372)
    results.append(check(p1 == p2, f"identical after 301 steps: {p1['drone_01']}"))
    print("\nRESULT:", "PASS" if all(results) else "FAIL", f"({sum(results)}/{len(results)})")
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
