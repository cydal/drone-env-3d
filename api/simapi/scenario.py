"""Scenario = reproducible description of a simulation run (YAML)."""
from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, Field


from .models import ActionLimits, EntityKind, Pose, Vec3


class CameraSpec(BaseModel):
    width: int = 320
    height: int = 240
    hfov: float = 1.396               # rad
    update_rate: float = 15.0
    pose: list[float] = Field(default_factory=lambda: [0.12, 0.0, -0.02, 0.0, 0.35, 0.0])  # x y z r p y
    depth: bool = True                # also emit a depth camera with the same intrinsics


class SensorMount(BaseModel):
    """A sensor attached to an entity at a configurable pose (x y z roll pitch yaw, body frame)."""
    name: str
    type: Literal["camera", "depth", "imu", "gps", "contact"]
    pose: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    params: dict[str, Any] = Field(default_factory=dict)   # camera/depth: width, height, hfov, update_rate, far


class AgentSpec(BaseModel):
    id: str
    type: EntityKind = "drone"
    template: str = "quadcopter"
    drone_type: str = "standard"      # see engine/gazebo/sdf.py DRONE_TYPES: standard | light | heavy
    spawn: Pose = Field(default_factory=Pose)
    observation: str = "state"        # profile name (built-in or scenario-defined)
    control: Literal["velocity", "waypoint"] = "velocity"   # highest action level accepted
    limits: ActionLimits | None = None   # None -> the drone type's defaults
    camera: CameraSpec | None = None  # legacy single forward camera (+depth); prefer `sensors`
    sensors: list[SensorMount] = Field(default_factory=list)   # configurable sensor mounts
    params: dict[str, Any] = Field(default_factory=dict)

    @property
    def camera_mounts(self) -> list[SensorMount]:
        """All image-producing mounts, including the legacy `camera:` block."""
        out = [m for m in self.sensors if m.type in ("camera", "depth")]
        if self.camera is not None and not out:
            cp = dict(width=self.camera.width, height=self.camera.height, hfov=self.camera.hfov, update_rate=self.camera.update_rate)
            out.append(SensorMount(name="camera", type="camera", pose=list(self.camera.pose), params=cp))
            if self.camera.depth:
                out.append(SensorMount(name="depth", type="depth", pose=list(self.camera.pose), params=cp))
        return out

    @property
    def has_rendering(self) -> bool:
        return bool(self.camera_mounts)


class TrajectorySpec(BaseModel):
    """Deterministic, environment-owned motion for non-agent entities."""
    type: Literal["circle", "line", "waypoints", "rotate", "static"] = "static"
    center: Vec3 = Field(default_factory=Vec3)
    radius: float = 10.0
    speed: float = 2.0                # m/s along the path
    start: Vec3 = Field(default_factory=Vec3)
    end: Vec3 = Field(default_factory=Vec3)
    waypoints: list[Vec3] = Field(default_factory=list)
    loop: bool = True                 # line: ping-pong / waypoints: cycle
    phase: float = 0.0                # seconds offset
    yaw_rate: float = 0.5             # rotate: rad/s about z at the spawn pose


class EntitySpec(BaseModel):
    """Dynamic non-agent entity (target, vehicle, moving obstacle)."""
    id: str
    type: EntityKind = "target"
    template: str = "target"
    spawn: Pose = Field(default_factory=Pose)
    trajectory: TrajectorySpec = Field(default_factory=TrajectorySpec)
    params: dict[str, Any] = Field(default_factory=dict)


class WorldSpec(BaseModel):
    name: str                         # world name inside the SDF
    file: str                         # path relative to sim/worlds
    bounds: dict[str, list[float]] | None = None   # {"x": [min,max], "y": [...], "z": [...]}
    areas: list[dict[str, Any]] = Field(default_factory=list)   # [{name, x:[..], y:[..]}] named regions


class EnvironmentSpec(BaseModel):
    wind: Vec3 = Field(default_factory=Vec3)
    time_of_day: str | float | None = None   # preset name (morning|day|evening|night) or hour 0-24
    visibility: float | None = None          # metres (fog); None = preset default


class SimulationSpec(BaseModel):
    step_size: float = 0.004          # seconds per physics step
    real_time_factor: float = 1.0
    seed: int = 0
    start_paused: bool = True
    mode: Literal["realtime", "stepped"] = "realtime"


class RegionSpec(BaseModel):
    x: list[float]
    y: list[float]
    z: list[float]


class AgentRandomization(BaseModel):
    region: RegionSpec | None = None      # spawn position sampled uniformly in the box
    yaw: bool = False                     # random heading


class EntityRandomization(BaseModel):
    region: RegionSpec | None = None      # for static-trajectory entities: random position
    routes: list["TrajectorySpec"] = Field(default_factory=list)   # pick one trajectory per seed


class RandomizationSpec(BaseModel):
    """Seeded, controlled randomisation of the initial conditions (brief 3b §20). The same
    scenario seed always reproduces the same samples; `reset(seed=...)` picks a different draw."""
    agents: dict[str, AgentRandomization] = Field(default_factory=dict)
    entities: dict[str, EntityRandomization] = Field(default_factory=dict)
    time_of_day: list[str] = Field(default_factory=list)   # choose one preset per seed (visual)


class EpisodeSpec(BaseModel):
    max_sim_time: float | None = None  # -> "timeout" event, episode completed
    terminate_on: list[str] = Field(default_factory=list)   # events that end the episode, e.g. ["collision"]


class LoggingSpec(BaseModel):
    enabled: bool = True
    realtime_hz: float = 10.0         # sampling rate in realtime mode (stepped: every step)


class Scenario(BaseModel):
    name: str
    description: str = ""
    world: WorldSpec
    agents: list[AgentSpec] = Field(default_factory=list)
    entities: list[EntitySpec] = Field(default_factory=list)
    observation_profiles: dict[str, list[str]] = Field(default_factory=dict)  # custom profiles
    environment: EnvironmentSpec = Field(default_factory=EnvironmentSpec)
    simulation: SimulationSpec = Field(default_factory=SimulationSpec)
    episode: EpisodeSpec = Field(default_factory=EpisodeSpec)
    logging: LoggingSpec = Field(default_factory=LoggingSpec)
    randomize: RandomizationSpec = Field(default_factory=RandomizationSpec)

    @property
    def rendering(self) -> bool:
        return any(a.has_rendering for a in self.agents)

    @classmethod
    def load(cls, path: Path) -> "Scenario":
        return cls.model_validate(yaml.safe_load(path.read_text()))

    def save(self, path: Path) -> None:
        path.write_text(yaml.safe_dump(self.model_dump(mode="json", exclude_none=True), sort_keys=False))


def list_scenarios(directory: Path) -> list[str]:
    return sorted(p.stem for p in directory.glob("*.yaml"))
