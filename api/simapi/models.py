"""Public data schema of the Simulation API.

These types are what browsers and external agents see. They contain no
Gazebo concepts; the engine layer maps to/from them.
"""
from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field


class Vec3(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Quat(BaseModel):
    """Orientation quaternion (x, y, z, w)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


class Pose(BaseModel):
    position: Vec3 = Field(default_factory=Vec3)
    orientation: Quat = Field(default_factory=Quat)


EntityKind = Literal["drone", "vehicle", "robot", "target", "obstacle", "building", "static", "dynamic", "unknown"]


class EntityInfo(BaseModel):
    entity_id: str
    kind: EntityKind = "unknown"
    is_agent: bool = False
    pose: Pose = Field(default_factory=Pose)


class EntityState(BaseModel):
    entity_id: str
    sim_time: float
    pose: Pose = Field(default_factory=Pose)
    linear_velocity: Vec3 = Field(default_factory=Vec3)
    angular_velocity: Vec3 = Field(default_factory=Vec3)
    linear_acceleration: Vec3 | None = None


class SimStatus(BaseModel):
    running: bool = False
    paused: bool = True
    sim_time: float = 0.0
    real_time: float = 0.0
    real_time_factor: float = 0.0
    iterations: int = 0
    world: str | None = None
    scenario: str | None = None
    seed: int | None = None
    step_size: float | None = None


class ImuReading(BaseModel):
    linear_acceleration: Vec3
    angular_velocity: Vec3
    orientation: Quat | None = None


class Observation(BaseModel):
    agent_id: str
    sim_time: float
    state: EntityState
    imu: ImuReading | None = None
    # Sensor frames are fetched via /agents/{id}/sensors/{name} (binary),
    # this lists what's available.
    sensors: list[str] = Field(default_factory=list)


class VelocityAction(BaseModel):
    """Body-frame linear velocity (m/s) and yaw rate (rad/s) setpoint."""
    type: Literal["velocity"] = "velocity"
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


class ArmAction(BaseModel):
    type: Literal["arm"] = "arm"
    armed: bool = True


Action = VelocityAction | ArmAction


class ActionEnvelope(BaseModel):
    action: Action


class StepRequest(BaseModel):
    steps: int = 1


class SpawnRequest(BaseModel):
    entity_id: str
    kind: EntityKind = "drone"
    template: str = "quadcopter"
    pose: Pose = Field(default_factory=Pose)
    params: dict[str, Any] = Field(default_factory=dict)


# ---- Scene description for the browser ------------------------------------

class Geometry(BaseModel):
    type: Literal["box", "cylinder", "sphere", "plane", "mesh", "unknown"]
    size: Vec3 | None = None          # box
    radius: float | None = None       # cylinder / sphere
    length: float | None = None       # cylinder
    normal: Vec3 | None = None        # plane
    uri: str | None = None            # mesh
    scale: Vec3 | None = None         # mesh


class Visual(BaseModel):
    name: str
    pose: Pose = Field(default_factory=Pose)    # relative to link
    geometry: Geometry
    color: list[float] | None = None            # rgba 0..1 (diffuse)


class LinkDesc(BaseModel):
    name: str
    pose: Pose = Field(default_factory=Pose)    # relative to model
    visuals: list[Visual] = Field(default_factory=list)


class ModelDesc(BaseModel):
    entity_id: str
    kind: EntityKind = "unknown"
    is_agent: bool = False
    pose: Pose = Field(default_factory=Pose)    # world frame
    links: list[LinkDesc] = Field(default_factory=list)
    is_static: bool = False


class SceneDesc(BaseModel):
    world: str
    models: list[ModelDesc]
