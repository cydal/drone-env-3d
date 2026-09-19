"""NavigationTask: point-to-point navigation on top of the Phase 2 simulation API.

    task = NavigationTask(Simulation("localhost"), TaskConfig(level="open"))
    obs, info = task.reset(seed=3)
    while True:
        obs, reward, terminated, truncated, info = task.step(action)   # action = [vx, vy, vz] m/s (world frame)
        if terminated or truncated: break

The task owns: start/target sampling, the observation vector, the reward, the
success/failure rules, per-episode metrics and (optionally) trajectory recording.
The simulator only ever sees velocity actions and answers with observations/events.
"""
from __future__ import annotations

import json
import math
import random
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from simclient import Simulation

AGENT = "drone_01"
TARGET = "target"

# ---------------------------------------------------------------------------
# configuration
# ---------------------------------------------------------------------------

@dataclass
class Region:
    """Axis-aligned box to sample positions from."""
    x: tuple[float, float]
    y: tuple[float, float]
    z: tuple[float, float]

    def sample(self, rng: random.Random) -> tuple[float, float, float]:
        return (rng.uniform(*self.x), rng.uniform(*self.y), rng.uniform(*self.z))


LEVELS: dict[str, dict[str, Any]] = {
    # start / target regions per level. Level worlds: see sim/scenarios/nav_level*.yaml
    "open":    {"scenario": "nav_level1_open",
                "start":  Region((-12, -4), (-12, 12), (4, 6)), "target": Region((6, 16), (-12, 12), (4, 6))},
    "pillars": {"scenario": "nav_level2_pillars",
                "start":  Region((-18, -12), (-14, 14), (3, 6)), "target": Region((12, 18), (-14, 14), (3, 6))},
    "city":    {"scenario": "nav_level3_city",
                "start":  Region((-45, -30), (-10, 25), (5, 10)), "target": Region((30, 45), (-10, 25), (5, 10))},
}


@dataclass
class RewardConfig:
    progress: float = 1.0          # per metre of distance reduction toward the target
    time_penalty: float = 0.01     # per step
    collision: float = 5.0         # subtracted on collision (episode ends)
    out_of_bounds: float = 5.0
    reached: float = 10.0
    action_penalty: float = 0.005  # * |action|^2 per step (smoothness)


@dataclass
class TaskConfig:
    level: str = "open"
    observation: str = "state"     # state | navigation | vision  (which simulator profile the agent has)
    success_radius: float = 1.0    # m
    max_steps: int = 200           # truncation
    action_repeat: int = 25        # physics iterations per task step (25 x 4 ms = 0.1 s)
    max_speed: float = 3.0         # m/s per axis, action scale
    terminate_on_collision: bool = True
    hold_altitude: bool = False    # if True, vz is forced to track the target altitude (2D task)
    reward: RewardConfig = field(default_factory=RewardConfig)
    start: Region | None = None    # overrides level defaults
    target: Region | None = None
    seed: int | None = None        # simulator seed (fixed per experiment); episode RNG is separate
    fixed_pairs: list[dict[str, Any]] | None = None   # evaluation: explicit [{start, target}, ...]

    def to_json(self) -> dict[str, Any]:
        d = asdict(self)
        return d


@dataclass
class EpisodeResult:
    episode_id: str
    seed: int
    start: tuple[float, float, float]
    target: tuple[float, float, float]
    success: bool
    termination: str                 # reached | collision | out_of_bounds | timeout | truncated
    steps: int
    sim_time: float
    return_: float
    final_distance: float
    path_length: float
    straight_line: float
    mean_abs_action_delta: float
    collisions: int

    @property
    def path_efficiency(self) -> float:
        return self.straight_line / self.path_length if self.path_length > 1e-6 else 0.0


# ---------------------------------------------------------------------------
# task
# ---------------------------------------------------------------------------

class NavigationTask:
    """Gym-like interface (obs, reward, terminated, truncated, info) over the Phase 2 API."""

    def __init__(self, sim: Simulation, cfg: TaskConfig, *, recorder: "TrajectoryRecorder | None" = None,
                 overlay: bool = True) -> None:
        self.sim = sim
        self.cfg = cfg
        self.level = LEVELS[cfg.level]
        self.start_region = cfg.start or self.level["start"]
        self.target_region = cfg.target or self.level["target"]
        self.recorder = recorder
        self.overlay = overlay
        self._rng = random.Random(0)
        self._episode_idx = -1
        self._loaded = False
        self.obs_dim = 12
        self.act_dim = 3
        self._reset_state()

    # ---- public --------------------------------------------------------------
    def reset(self, *, seed: int | None = None, pair: dict[str, Any] | None = None) -> tuple[np.ndarray, dict]:
        self._episode_idx += 1
        if seed is not None:
            self._rng = random.Random(seed)
        ep_seed = seed if seed is not None else self._rng.randrange(1 << 30)
        self._ensure_loaded()
        # world rewind (fast: no cameras in the nav scenarios); simulator seed stays fixed
        self.sim.reset()
        if pair is None and self.cfg.fixed_pairs:
            pair = self.cfg.fixed_pairs[self._episode_idx % len(self.cfg.fixed_pairs)]
        if pair is not None:
            start, target = tuple(pair["start"]), tuple(pair["target"])
        else:
            start, target = self.start_region.sample(self._rng), self.target_region.sample(self._rng)
            while _dist(start, target) < 3.0:
                target = self.target_region.sample(self._rng)
        self.sim.set_pose(TARGET, target)
        self.sim.set_pose(AGENT, start, yaw=self._rng.uniform(-math.pi, math.pi) if pair is None else 0.0)
        # arm + hold so the controller keeps the drone airborne at the teleported pose
        self.sim.step(agent=AGENT, action=self.sim.arm(True), steps=1)
        self.sim.step(agent=AGENT, action=self.sim.hold(), steps=20, observe=False)
        self._reset_state()
        self.start, self.target, self.ep_seed = start, target, ep_seed
        self.episode_id = self.sim.episode().episode_id
        obs, st = self._observe()
        self.prev_distance = _dist(self._pos(st), target)
        self.prev_pos = self._pos(st)
        self.straight_line = _dist(start, target)
        if self.recorder:
            self.recorder.begin(self)
        self._push_overlay(obs, None, 0.0, "running")
        return obs, self._info(st)

    def step(self, action: np.ndarray | list[float]) -> tuple[np.ndarray, float, bool, bool, dict]:
        a = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        vx, vy, vz = (float(v) * self.cfg.max_speed for v in a)
        if self.cfg.hold_altitude:
            vz = float(np.clip(1.0 * (self.target[2] - self.prev_pos[2]), -1.0, 1.0))
        res = self.sim.step(agent=AGENT, action=self.sim.velocity(vx, vy, vz, frame="world"), steps=self.cfg.action_repeat)
        self.steps += 1
        obs, st = self._observe(res.observations.get(AGENT))
        pos = self._pos(st)
        dist = _dist(pos, self.target)
        events = [e.event for e in res.events]
        collided = "collision" in events
        oob = "out_of_bounds" in events
        reached = dist < self.cfg.success_radius

        r = self.cfg.reward
        reward = r.progress * (self.prev_distance - dist) - r.time_penalty - r.action_penalty * float(np.sum(a * a))
        terminated, termination = False, None
        if reached:
            reward += r.reached; terminated, termination = True, "reached"
        elif collided and self.cfg.terminate_on_collision:
            reward -= r.collision; terminated, termination = True, "collision"
        elif oob:
            reward -= r.out_of_bounds; terminated, termination = True, "out_of_bounds"
        elif res.episode.status in ("completed", "terminated", "failed"):
            terminated, termination = True, "timeout"
        truncated = (not terminated) and self.steps >= self.cfg.max_steps
        if truncated:
            termination = "truncated"

        self.path_length += _dist(pos, self.prev_pos)
        if self.prev_action is not None:
            self.action_deltas.append(float(np.abs(a - self.prev_action).mean()))
        self.collisions += int(collided)
        self.return_ += reward
        self.prev_distance, self.prev_pos, self.prev_action = dist, pos, a
        info = self._info(st, distance=dist, events=events, reward=reward, termination=termination)
        if self.recorder:
            self.recorder.record(obs, a, reward, st, events, info)
        if terminated or truncated:
            self.result = EpisodeResult(
                episode_id=self.episode_id, seed=self.ep_seed, start=self.start, target=self.target,
                success=termination == "reached", termination=termination or "unknown", steps=self.steps,
                sim_time=st["sim_time"], return_=self.return_, final_distance=dist, path_length=self.path_length,
                straight_line=self.straight_line,
                mean_abs_action_delta=float(np.mean(self.action_deltas)) if self.action_deltas else 0.0,
                collisions=self.collisions)
            info["result"] = self.result
            if self.recorder:
                self.recorder.end(self.result)
        self._push_overlay(obs, a, reward, termination or "running", dist)
        return obs, float(reward), terminated, truncated, info

    def observation_space_info(self) -> dict[str, Any]:
        return {"dim": self.obs_dim, "layout": ["rel_target_xyz(3)", "vel_world_xyz(3)", "heading_sin_cos(2)",
                                                 "altitude(1)", "distance(1)", "target_dir_body_xy(2)"],
                "profile": self.cfg.observation}

    # ---- internals -------------------------------------------------------------
    @property
    def scenario_name(self) -> str:
        """Same world per level; the observation mode selects the agent's profile/sensors:
        <level>            -> observation: state       (privileged)
        <level>_navigation -> observation: navigation  (gps, imu, body velocity)
        <level>_vision     -> observation: vision_nav  (camera, depth, imu, gps, velocity)"""
        base = self.level["scenario"]
        return base if self.cfg.observation == "state" else f"{base}_{self.cfg.observation}"

    def _ensure_loaded(self) -> None:
        st = self.sim.status()
        if self._loaded and st.get("running") and st.get("scenario") == self.scenario_name:
            return
        self.sim.reset(scenario=self.scenario_name, seed=self.cfg.seed, mode="stepped")
        comps = self.sim.agent(AGENT)["observation_space"]["components"]
        if self.cfg.observation == "state" and "state" not in comps:
            raise RuntimeError(f"scenario {self.scenario_name} does not expose privileged state")
        if self.cfg.observation != "state" and "gps" not in comps:
            raise RuntimeError(f"scenario {self.scenario_name} lacks gps; navigation/vision modes need it")
        self._loaded = True

    def _reset_state(self) -> None:
        self.steps = 0
        self.return_ = 0.0
        self.path_length = 0.0
        self.collisions = 0
        self.prev_action: np.ndarray | None = None
        self.action_deltas: list[float] = []
        self.result: EpisodeResult | None = None
        self.start = self.target = (0.0, 0.0, 0.0)
        self.prev_pos = (0.0, 0.0, 0.0)
        self.prev_distance = 0.0
        self.straight_line = 1.0
        self.ep_seed = 0
        self.episode_id = ""

    def _observe(self, obs_json: dict | None = None):
        """Build the observation vector under the configured observation mode.

        state:      privileged pose + velocity from `observation.state`
        navigation: GPS -> local ENU metres, IMU-derived heading, body velocity rotated by heading
        vision:     (frames are fetched by the policy separately) plus IMU + target info
        Target position is always given relative to the drone (the task is "reach the target").
        """
        o = obs_json if obs_json is not None else self.sim.observe(AGENT)
        mode = self.cfg.observation
        if mode == "state" or o.get("state"):
            st = o["state"]
            p = st["pose"]["position"]; v = st["linear_velocity"]; q = st["pose"]["orientation"]
            pos = (p["x"], p["y"], p["z"]); vel = (v["x"], v["y"], v["z"])
            yaw = _yaw(q)
        else:
            g = o.get("gps"); imu = o.get("imu"); bv = o.get("velocity", {}).get("linear", {"x": 0, "y": 0, "z": 0})
            if g is None:
                raise RuntimeError(f"observation mode {mode!r} needs the agent's profile to include gps")
            pos = _enu_from_gps(g)
            yaw = _yaw(imu["orientation"]) if imu and imu.get("orientation") else 0.0
            c, s_ = math.cos(yaw), math.sin(yaw)
            vel = (c * bv["x"] - s_ * bv["y"], s_ * bv["x"] + c * bv["y"], bv["z"])
        rel = (self.target[0] - pos[0], self.target[1] - pos[1], self.target[2] - pos[2])
        dist = math.sqrt(sum(r * r for r in rel))
        c, s_ = math.cos(yaw), math.sin(yaw)
        body_dir = ((c * rel[0] + s_ * rel[1]) / max(dist, 1e-6), (-s_ * rel[0] + c * rel[1]) / max(dist, 1e-6))
        vec = np.array([*rel, *vel, math.sin(yaw), math.cos(yaw), pos[2], dist, *body_dir], dtype=np.float32)
        st_like = {"sim_time": o["sim_time"], "pos": pos, "vel": vel, "yaw": yaw, "raw": o}
        return vec, st_like

    @staticmethod
    def _pos(st) -> tuple[float, float, float]:
        return tuple(st["pos"])

    def _info(self, st, **extra) -> dict[str, Any]:
        d = {"sim_time": st["sim_time"], "position": st["pos"], "velocity": st["vel"], "target": self.target,
             "steps": self.steps, "episode_id": self.episode_id, "seed": self.ep_seed}
        d.update(extra)
        return d

    def _push_overlay(self, obs, action, reward, status, dist=None) -> None:
        if not self.overlay:
            return
        try:
            self.sim.overlay({
                "task": "navigation", "level": self.cfg.level, "agent": AGENT, "target": list(self.target),
                "start": list(self.start), "distance": dist if dist is not None else float(obs[9]),
                "success_radius": self.cfg.success_radius, "step": self.steps, "max_steps": self.cfg.max_steps,
                "reward": float(reward), "return": float(self.return_), "status": status,
                "action": [float(v) for v in action] if action is not None else None,
                "observation": [round(float(v), 3) for v in obs.tolist()], "observation_mode": self.cfg.observation,
            })
        except Exception:
            pass  # overlay is best-effort; never let display break the task


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _dist(a, b) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _yaw(q) -> float:
    return math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))


_ORIGIN = (47.3977, 8.5456, 488.0)   # world spherical_coordinates in the nav worlds


def _enu_from_gps(g) -> tuple[float, float, float]:
    """Local ENU metres from lat/lon/alt, relative to the world origin (equirectangular, fine at this scale)."""
    lat0, lon0, alt0 = _ORIGIN
    R = 6378137.0
    x = math.radians(g["longitude_deg"] - lon0) * R * math.cos(math.radians(lat0))
    y = math.radians(g["latitude_deg"] - lat0) * R
    return (x, y, g["altitude"] - alt0)


# ---------------------------------------------------------------------------
# trajectory recording (datasets are first-class artefacts)
# ---------------------------------------------------------------------------

class TrajectoryRecorder:
    """Writes one .npz per episode plus an index.jsonl:
        obs[T+1, D], action[T, A], reward[T], sim_time[T+1], position[T+1, 3], velocity[T+1, 3],
        events (list per step), target[3], start[3], result{...}
    obs_t, action_t, obs_{t+1} alignment is exactly what dynamics / world-model training needs.
    """

    def __init__(self, directory: Path, name: str, *, meta: dict[str, Any] | None = None) -> None:
        self.dir = Path(directory) / name
        self.dir.mkdir(parents=True, exist_ok=True)
        self.index = self.dir / "index.jsonl"
        (self.dir / "meta.json").write_text(json.dumps({"created": time.time(), **(meta or {})}, indent=2, default=str))
        self.n = sum(1 for _ in self.index.open()) if self.index.exists() else 0
        self._cur: dict[str, list] | None = None
        self._task: NavigationTask | None = None

    def begin(self, task: NavigationTask) -> None:
        self._task = task
        first_obs, st = task._observe()
        self._cur = {"obs": [first_obs], "action": [], "reward": [], "sim_time": [st["sim_time"]],
                     "position": [st["pos"]], "velocity": [st["vel"]], "events": []}

    def record(self, obs, action, reward, st, events, info) -> None:
        if self._cur is None:
            return
        c = self._cur
        c["obs"].append(obs); c["action"].append(action); c["reward"].append(reward)
        c["sim_time"].append(st["sim_time"]); c["position"].append(st["pos"]); c["velocity"].append(st["vel"])
        c["events"].append(events)

    def end(self, result: EpisodeResult) -> Path:
        assert self._cur is not None and self._task is not None
        c = self._cur
        path = self.dir / f"episode_{self.n:05d}.npz"
        np.savez_compressed(
            path, obs=np.asarray(c["obs"], np.float32), action=np.asarray(c["action"], np.float32),
            reward=np.asarray(c["reward"], np.float32), sim_time=np.asarray(c["sim_time"], np.float64),
            position=np.asarray(c["position"], np.float32), velocity=np.asarray(c["velocity"], np.float32),
            events=np.asarray(json.dumps(c["events"])), target=np.asarray(result.target, np.float32),
            start=np.asarray(result.start, np.float32), result=np.asarray(json.dumps(asdict(result))))
        with self.index.open("a") as f:
            f.write(json.dumps({"file": path.name, **asdict(result)}) + "\n")
        self.n += 1
        self._cur = None
        return path
