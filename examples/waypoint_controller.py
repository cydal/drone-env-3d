"""Integration test controller (Phase 1D): take off -> waypoint -> hover -> land.

Knows nothing about Gazebo. Talks only to the Simulation API through SimulationClient.
Run:  python examples/waypoint_controller.py [--agent drone_01] [--scenario test_city_two_drones]
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "client"))
from simclient import SimulationClient  # noqa: E402


def pos(obs) -> tuple[float, float, float]:
    p = obs["state"]["pose"]["position"]
    return p["x"], p["y"], p["z"]


def yaw_of(obs) -> float:
    q = obs["state"]["pose"]["orientation"]
    return math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))


def world_to_body(vx: float, vy: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


def fly_to(sim: SimulationClient, agent: str, target, *, speed=2.0, tol=0.35, timeout=60.0, log=print) -> bool:
    """Proportional velocity controller toward a world-frame target position."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        obs = sim.get_observation(agent)
        x, y, z = pos(obs)
        dx, dy, dz = target[0] - x, target[1] - y, target[2] - z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < tol:
            return True
        gain = 1.2
        vx, vy, vz = gain * dx, gain * dy, gain * dz
        norm = math.sqrt(vx * vx + vy * vy + vz * vz)
        if norm > speed:
            vx, vy, vz = (v * speed / norm for v in (vx, vy, vz))
        bx, by = world_to_body(vx, vy, yaw_of(obs))
        sim.send_velocity(agent, vx=bx, vy=by, vz=vz, yaw_rate=0.0)
        time.sleep(0.05)
    return False


def hover(sim: SimulationClient, agent: str, seconds: float) -> None:
    t0 = time.time()
    while time.time() - t0 < seconds:
        sim.send_velocity(agent)  # zero twist = hold position
        time.sleep(0.1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--agent", default="drone_01")
    ap.add_argument("--scenario", default=None, help="load this scenario first (otherwise use the running one)")
    ap.add_argument("--waypoint", default="8,4,6", help="x,y,z of the waypoint")
    args = ap.parse_args()

    sim = SimulationClient(args.host, args.port)
    if args.scenario:
        print(f"loading scenario {args.scenario}")
        sim.load_scenario(args.scenario)
    sim.wait_until_ready()
    sim.resume()

    start = pos(sim.get_observation(args.agent))
    wp = tuple(float(v) for v in args.waypoint.split(","))
    print(f"[{args.agent}] start at {tuple(round(v, 2) for v in start)}")

    sim.arm(args.agent, True)
    steps = [
        ("take off", (start[0], start[1], 4.0)),
        ("to waypoint", wp),
    ]
    for label, target in steps:
        print(f"[{args.agent}] {label} -> {target}")
        if not fly_to(sim, args.agent, target):
            print("  timeout"); return 1
        print(f"  reached  ({tuple(round(v, 2) for v in pos(sim.get_observation(args.agent)))})")

    print(f"[{args.agent}] hover 3 s")
    hover(sim, args.agent, 3.0)

    print(f"[{args.agent}] land")
    fly_to(sim, args.agent, (wp[0], wp[1], 0.6), speed=1.0, tol=0.25)
    fly_to(sim, args.agent, (wp[0], wp[1], 0.15), speed=0.5, tol=0.12, timeout=15)
    sim.arm(args.agent, False)  # motors off
    time.sleep(1.0)
    final = pos(sim.get_observation(args.agent))
    print(f"[{args.agent}] landed at {tuple(round(v, 2) for v in final)}")
    ok = math.hypot(final[0] - wp[0], final[1] - wp[1]) < 1.0 and final[2] < 0.5
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
