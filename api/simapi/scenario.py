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


class AgentSpec(BaseModel):
    id: str
    type: EntityKind = "drone"
    template: str = "quadcopter"
    spawn: Pose = Field(default_factory=Pose)
    observation: str = "state"        # profile name (built-in or scenario-defined)
    control: Literal["velocity", "waypoint"] = "velocity"   # highest action level accepted
    limits: ActionLimits = Field(default_factory=ActionLimits)
    camera: CameraSpec | None = None  # present -> rendering sensors are attached
    params: dict[str, Any] = Field(default_factory=dict)


class TrajectorySpec(BaseModel):
    """Deterministic, environment-owned motion for non-agent entities."""
    type: Literal["circle", "line", "waypoints", "static"] = "static"
    center: Vec3 = Field(default_factory=Vec3)
    radius: float = 10.0
    speed: float = 2.0                # m/s along the path
    start: Vec3 = Field(default_factory=Vec3)
    end: Vec3 = Field(default_factory=Vec3)
    waypoints: list[Vec3] = Field(default_factory=list)
    loop: bool = True                 # line: ping-pong / waypoints: cycle
    phase: float = 0.0                # seconds offset


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


class EnvironmentSpec(BaseModel):
    wind: Vec3 = Field(default_factory=Vec3)
    time_of_day: float | None = None
    visibility: float | None = None


class SimulationSpec(BaseModel):
    step_size: float = 0.004          # seconds per physics step
    real_time_factor: float = 1.0
    seed: int = 0
    start_paused: bool = True
    mode: Literal["realtime", "stepped"] = "realtime"


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

    @property
    def rendering(self) -> bool:
        return any(a.camera is not None for a in self.agents)

    @classmethod
    def load(cls, path: Path) -> "Scenario":
        return cls.model_validate(yaml.safe_load(path.read_text()))

    def save(self, path: Path) -> None:
        path.write_text(yaml.safe_dump(self.model_dump(mode="json", exclude_none=True), sort_keys=False))


def list_scenarios(directory: Path) -> list[str]:
    return sorted(p.stem for p in directory.glob("*.yaml"))
