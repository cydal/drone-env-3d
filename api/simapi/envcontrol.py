"""Environment-owned motion for non-agent entities (targets, vehicles).

Deterministic functions of simulation time -> pose. No intelligence here; this
is the world changing on its own so agents have something to observe/pursue.
"""
from __future__ import annotations

import math

from .models import Pose, Quat, Vec3
from .scenario import EntitySpec, TrajectorySpec


def _yaw_quat(yaw: float) -> Quat:
    return Quat(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def _lerp(a: Vec3, b: Vec3, s: float) -> Vec3:
    return Vec3(x=a.x + (b.x - a.x) * s, y=a.y + (b.y - a.y) * s, z=a.z + (b.z - a.z) * s)


def _dist(a: Vec3, b: Vec3) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def pose_at(spec: TrajectorySpec, t: float, spawn: Pose) -> Pose:
    t = t + spec.phase
    if spec.type == "static":
        return spawn

    if spec.type == "rotate":
        return Pose(position=spawn.position, orientation=_yaw_quat(spec.yaw_rate * t))

    if spec.type == "circle":
        r = max(spec.radius, 1e-3)
        omega = spec.speed / r
        a = omega * t
        pos = Vec3(x=spec.center.x + r * math.cos(a), y=spec.center.y + r * math.sin(a), z=spec.center.z)
        return Pose(position=pos, orientation=_yaw_quat(a + math.pi / 2))

    if spec.type == "line":
        length = _dist(spec.start, spec.end)
        if length < 1e-6:
            return Pose(position=spec.start)
        s = spec.speed * t / length
        if spec.loop:
            s = s % 2.0
            forward = s <= 1.0
            s = s if forward else 2.0 - s
        else:
            forward = True
            s = min(1.0, s)
        a, b = (spec.start, spec.end) if forward else (spec.end, spec.start)
        yaw = math.atan2(b.y - a.y, b.x - a.x)
        return Pose(position=_lerp(spec.start, spec.end, s), orientation=_yaw_quat(yaw))

    if spec.type == "waypoints":
        pts = spec.waypoints
        if not pts:
            return spawn
        if len(pts) == 1:
            return Pose(position=pts[0])
        segs = list(zip(pts, pts[1:] + ([pts[0]] if spec.loop else [])))
        lengths = [_dist(a, b) for a, b in segs]
        total = sum(lengths)
        if total < 1e-6:
            return Pose(position=pts[0])
        d = spec.speed * t
        d = d % total if spec.loop else min(d, total - 1e-9)
        for (a, b), L in zip(segs, lengths):
            if d <= L or L == 0:
                s = d / L if L else 0.0
                yaw = math.atan2(b.y - a.y, b.x - a.x)
                return Pose(position=_lerp(a, b, s), orientation=_yaw_quat(yaw))
            d -= L
        return Pose(position=pts[-1])
    return spawn


class TrajectoryController:
    """Drives every scenario entity along its trajectory via engine.set_pose."""

    def __init__(self, entities: list[EntitySpec]) -> None:
        self.entities = [e for e in entities if e.trajectory.type != "static"]

    def targets(self, sim_time: float) -> dict[str, Pose]:
        return {e.id: pose_at(e.trajectory, sim_time, e.spawn) for e in self.entities}
