"""Observation profiles: which information an agent is allowed to see.

Built-in profiles; scenarios may define more under `observation_profiles`.
Components (see models.ObservationComponent):

  state          privileged ground truth of the agent itself (pose, velocities, accel)
  velocity       body-frame velocity as an onboard estimator would give it
  imu            IMU sensor
  gps            NavSat sensor (lat/lon/alt + ENU velocity)
  camera         RGB frame reference
  depth          depth frame reference
  nearby_agents  relative positions of other agents within `nearby_radius`
  swarm_state    privileged ground truth of *all* agents (centralised observation)
"""
from __future__ import annotations

BUILTIN_PROFILES: dict[str, list[str]] = {
    "state":      ["state"],
    "navigation": ["gps", "imu", "velocity"],
    "vision":     ["camera", "depth", "imu"],
    "minimal":    ["camera", "imu"],
    "shared":     ["state", "nearby_agents"],
    "central":    ["state", "swarm_state"],
    "full":       ["state", "velocity", "imu", "gps", "camera", "depth", "nearby_agents", "swarm_state"],
}

VALID_COMPONENTS = {"state", "velocity", "imu", "gps", "camera", "depth", "nearby_agents", "swarm_state"}
NEARBY_RADIUS_M = 30.0


def resolve_profile(name: str, custom: dict[str, list[str]] | None = None) -> list[str]:
    table = {**BUILTIN_PROFILES, **(custom or {})}
    if name not in table:
        raise ValueError(f"unknown observation profile {name!r}; known: {sorted(table)}")
    comps = table[name]
    bad = set(comps) - VALID_COMPONENTS
    if bad:
        raise ValueError(f"profile {name!r} has unknown components {sorted(bad)}")
    return list(comps)
