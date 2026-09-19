"""Shared bits for the example controllers (argument parsing, small helpers)."""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "client"))
from simclient import Simulation  # noqa: E402


def parser(desc: str, scenario: str = "phase2_city") -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--scenario", default=scenario)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--no-reset", action="store_true", help="use the running episode instead of resetting")
    return ap


def connect(args) -> Simulation:
    return Simulation(args.host, args.port)


def p_control(pos, target, gain=1.2, speed=2.0):
    """World-frame velocity toward target, capped at `speed`."""
    dx, dy, dz = target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]
    vx, vy, vz = gain * dx, gain * dy, gain * dz
    n = math.sqrt(vx * vx + vy * vy + vz * vz)
    if n > speed:
        vx, vy, vz = vx * speed / n, vy * speed / n, vz * speed / n
    return vx, vy, vz, math.sqrt(dx * dx + dy * dy + dz * dz)


def fmt(p) -> str:
    return "(" + ", ".join(f"{v:.2f}" for v in p) + ")"
