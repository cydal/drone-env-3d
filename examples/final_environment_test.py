"""Brief 3b §43 - the definitive environment test, with zero AI.

Loads a scenario, spawns 3 drones / 2 vehicles / 1 moving target into a world with 20+
static structures, then exercises: human-style control (velocity commands through the same
public API the browser keyboard uses), a Python-controlled drone, a scripted drone, vehicles
and target on their own routes, pause, step, inspect, sensors, snapshot, restore, record,
replay, and reset with the same seed. Prints a checklist. Run with the API up:

    .venv/bin/python examples/final_environment_test.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "client"))
from simclient import Simulation  # noqa: E402


def check(ok, label: str) -> bool:
    ok = bool(ok)
    print(("  ✅ " if ok else "  ❌ ") + label)
    return ok


def pos(sim, e):
    p = sim.entity_state(e)["pose"]["position"]
    return (round(p["x"], 4), round(p["y"], 4), round(p["z"], 4))


def main() -> int:
    sim = Simulation(timeout=180)
    R = []
    print("1. start + load a scenario")
    ep = sim.reset(scenario="moving_traffic", seed=31, mode="stepped")
    R.append(check(sim.status()["running"] and ep.mode == "stepped", f"scenario moving_traffic loaded (episode {ep.episode_id})"))

    print("2. spawn 3 drones, 2 vehicles, 1 moving target; 20+ static structures present")
    sim.spawn("drone_02", drone_type="light", position=(0, -55, 0.3), observation="navigation")
    sim.spawn("drone_03", drone_type="heavy", position=(25, -55, 0.3), observation="state")
    sim.spawn("target_01", template="target", position=(90, -70, 12), trajectory={"type": "circle", "center": {"x": 90, "y": -70, "z": 12}, "radius": 15, "speed": 3}, collide=False)
    ents = sim.entities(); cats = {}
    for e in ents:
        cats[e["category"]] = cats.get(e["category"], 0) + 1
    R.append(check(cats.get("drone", 0) == 3 and cats.get("vehicle", 0) >= 2 and cats.get("target", 0) == 1
                   and cats.get("building", 0) + cats.get("infrastructure", 0) + cats.get("obstacle", 0) >= 20,
                   f"entities by category: {dict(sorted(cats.items()))}"))

    print("3. control: human-style (drone_01), Python (drone_02), scripted trajectory (drone_03)")
    for a in ("drone_01", "drone_02", "drone_03"):
        sim.act(a, sim.arm(True))
    sim.step(steps=1)
    # human style: velocity commands like the browser keyboard sends
    sim.step(actions={"drone_01": sim.velocity(0, 0, 1.5)}, steps=250)
    # python controller: waypoint action; scripted: a fixed sequence of waypoints
    script = [(25, -55, 8), (35, -45, 8), (25, -35, 8)]
    r = sim.step(actions={"drone_02": sim.waypoint(0, -55, 6, speed=2.0), "drone_03": sim.waypoint(*script[0], speed=2.0)}, steps=750)
    assert not r.rejected_actions, r.rejected_actions
    r = sim.step(actions={"drone_03": sim.waypoint(*script[1], speed=2.0)}, steps=750)
    assert not r.rejected_actions, r.rejected_actions
    z = {a: pos(sim, a)[2] for a in ("drone_01", "drone_02", "drone_03")}
    R.append(check(all(v > 3.0 for v in z.values()), f"all three drones airborne: { {k: round(v, 1) for k, v in z.items()} }"))
    v0 = pos(sim, "vehicle_01"); t0 = pos(sim, "target_01")
    sim.step(steps=250)
    R.append(check(pos(sim, "vehicle_01") != v0 and pos(sim, "target_01") != t0, "vehicles and target move on their own routes"))

    print("4. observe: telemetry, sensor feeds, events, simulation time")
    o = sim.observe("drone_01"); fr = sim.frame("drone_01", "front_cam"); d = sim.entity_detail("drone_03")
    R.append(check(o.get("state") is not None and fr is not None and fr.shape[2] == 3, f"observation + camera frame {fr.shape if fr is not None else None}"))
    R.append(check(d["physical"]["drone_type"] == "heavy" and d["dimensions"] is not None, f"inspector detail: {d['type_label']} {d['dimensions']}"))
    st = sim.status()
    R.append(check(st["sim_time"] > 7.0 and st["iterations"] == round(st["sim_time"] / 0.004), f"simulation clock explicit: t={st['sim_time']:.3f}s iter={st['iterations']}"))

    print("5. pause / step")
    it = sim.status()["iterations"]; r = sim.step(steps=25)
    R.append(check(r.status["iterations"] == it + 25 and r.status["paused"], "step advances exactly 25 iterations and stays paused"))

    print("6. record rows, snapshot, diverge, restore")
    rec = sim.recording_start(observations=True, states=True)
    snap = sim.snapshot("final-test")
    captured = {e: pos(sim, e) for e in ("drone_01", "drone_02", "drone_03", "vehicle_01", "target_01")}
    sim.step(actions={"drone_01": sim.velocity(1.5, 1.0, 0.0)}, steps=250)
    diverged = pos(sim, "drone_01") != captured["drone_01"]
    res = sim.restore(snap["snapshot_id"])
    restored = {e: pos(sim, e) for e in captured}
    R.append(check(diverged and restored == captured and res["divergence"]["max_position_m"] < 1e-6,
                   f"snapshot restored exactly after divergence (max divergence {res['divergence']['max_position_m']:.1e} m)"))
    rows = sim.recording_rows(rec["recording_id"], 0, 5)
    R.append(check(len(rows) > 0 and rows[0]["states"] and rows[0]["observations"], f"recording rows pulled ({len(rows)} shown) with states + observations"))

    print("7. replay the run")
    # restore started a fresh episode (and a fresh recording); keep flying, then replay *that* recording
    sim.step(actions={"drone_01": sim.velocity(-1.0, 0.5, 0.2)}, steps=250)
    end = pos(sim, "drone_01"); last_it = sim.status()["iterations"]
    current_rec = sim.world_state()["recording"]["recording_id"]
    rep = sim.replay(current_rec)
    R.append(check(rep["replayed_to_iteration"] == last_it and pos(sim, "drone_01") == end,
                   f"replay of the current recording reproduces its final state exactly ({last_it} iterations)"))

    print("8. reset with the same seed reproduces the initial conditions")
    ep1 = sim.reset(seed=31); a = pos(sim, "drone_01"); rz1 = sim.world_state()["randomization"]
    ep2 = sim.reset(seed=31); b = pos(sim, "drone_01"); rz2 = sim.world_state()["randomization"]
    R.append(check(a == b and rz1 == rz2 and ep1.episode_id != ep2.episode_id, f"same seed -> same start {a}, same randomisation draw"))

    print(f"\nRESULT: {'PASS' if all(R) else 'FAIL'} ({sum(R)}/{len(R)})")
    return 0 if all(R) else 1


if __name__ == "__main__":
    sys.exit(main())
