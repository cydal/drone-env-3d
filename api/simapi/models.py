"""Public data schema of the Simulation API (Phase 2).

These types are what browsers and external agents see. They contain no Gazebo
concepts; the engine layer maps to/from them.

Two kinds of information are deliberately kept apart:

* **ground-truth state** (`EntityState`) — privileged, internal, available via
  `/entities/{id}` for tooling and debugging;
* **observations** (`Observation`) — what an agent is *allowed* to see,
  determined by its observation profile (see `observation.py`).
"""
from __future__ import annotations

import math
from typing import Any, Literal

from pydantic import BaseModel, Field, field_validator, model_validator


# ---------------------------------------------------------------------------
# geometry primitives
# ---------------------------------------------------------------------------

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


EntityKind = Literal["drone", "vehicle", "robot", "target", "obstacle", "building",
                     "static", "dynamic", "unknown"]


# ---------------------------------------------------------------------------
# ground truth
# ---------------------------------------------------------------------------

class EntityInfo(BaseModel):
    entity_id: str
    kind: EntityKind = "unknown"
    is_agent: bool = False
    pose: Pose = Field(default_factory=Pose)
    category: str = "obstacle"       # drone | vehicle | target | dynamic | building | infrastructure | obstacle | terrain


class EntityDetail(BaseModel):
    """Everything the inspector (or an external tool) may want about one entity."""
    entity_id: str
    kind: EntityKind
    category: str                              # drone | vehicle | target | building | infrastructure | obstacle | dynamic | terrain
    is_agent: bool
    is_static: bool
    template: str | None = None
    type_label: str | None = None              # e.g. "Quadrotor (light, agile)", "Ground vehicle", "Building"
    dimensions: Vec3 | None = None             # bounding box of the visuals (m)
    collision: bool = True
    trajectory: str | None = None              # "circle r=8 @2 m/s", "line ...", None
    control: str | None = None                 # agents: "external" | "waypoint" | "idle"
    sensors: list[str] = Field(default_factory=list)
    sensor_mounts: list[dict[str, Any]] = Field(default_factory=list)
    physical: dict[str, Any] = Field(default_factory=dict)   # mass, limits, drone_type ...
    state: EntityState | None = None


class EntityState(BaseModel):
    """Privileged simulator state (world frame unless noted)."""
    entity_id: str
    sim_time: float
    pose: Pose = Field(default_factory=Pose)
    linear_velocity: Vec3 = Field(default_factory=Vec3)          # world frame
    angular_velocity: Vec3 = Field(default_factory=Vec3)         # body frame
    linear_acceleration: Vec3 | None = None                      # world frame, differentiated


SimMode = Literal["realtime", "stepped"]


class SimStatus(BaseModel):
    running: bool = False
    paused: bool = True
    mode: SimMode = "realtime"
    speed: float = 1.0                       # target real-time factor in realtime mode (0 = unlimited)
    sim_time: float = 0.0
    real_time: float = 0.0
    real_time_factor: float = 0.0
    iterations: int = 0
    world: str | None = None
    scenario: str | None = None
    seed: int | None = None
    step_size: float | None = None
    episode: "Episode | None" = None


# ---------------------------------------------------------------------------
# sensors / observations
# ---------------------------------------------------------------------------

class ImuReading(BaseModel):
    linear_acceleration: Vec3          # body frame, includes gravity reaction
    angular_velocity: Vec3             # body frame
    orientation: Quat | None = None


class GpsReading(BaseModel):
    latitude_deg: float
    longitude_deg: float
    altitude: float
    velocity_enu: Vec3 = Field(default_factory=Vec3)


class BodyVelocity(BaseModel):
    """Velocity as an onboard estimator would report it: body frame."""
    linear: Vec3 = Field(default_factory=Vec3)
    angular: Vec3 = Field(default_factory=Vec3)


class NearbyAgent(BaseModel):
    agent_id: str
    relative_position: Vec3            # world-frame offset from observer
    distance: float
    velocity: Vec3 | None = None       # world frame, if profile allows


class SensorFrameRef(BaseModel):
    """Descriptor for a high-bandwidth frame that is *not* inlined in JSON."""
    name: str
    type: Literal["rgb", "depth"]
    width: int
    height: int
    encoding: str                      # e.g. "rgb8", "depth32f"
    sim_time: float | None = None
    seq: int = 0
    url: str                           # HTTP path returning the encoded frame
    stream: str                        # WebSocket path streaming encoded frames


ObservationComponent = Literal["state", "velocity", "imu", "gps", "camera", "depth",
                               "nearby_agents", "swarm_state"]


class Observation(BaseModel):
    """What an agent sees. Fields are present only if its profile includes them."""
    agent_id: str
    sim_time: float
    iteration: int = 0
    profile: str
    # privileged
    state: EntityState | None = None
    swarm_state: list[EntityState] | None = None
    # onboard-style
    velocity: BodyVelocity | None = None
    imu: ImuReading | None = None
    gps: GpsReading | None = None
    nearby_agents: list[NearbyAgent] | None = None
    # high-bandwidth (references only)
    frames: list[SensorFrameRef] = Field(default_factory=list)
    # bookkeeping
    grounded: bool | None = None
    armed: bool | None = None
    control_mode: str | None = None


# ---------------------------------------------------------------------------
# actions
# ---------------------------------------------------------------------------

class ActionLimits(BaseModel):
    max_speed_xy: float = 6.0          # m/s
    max_speed_z: float = 3.0           # m/s
    max_yaw_rate: float = 1.5          # rad/s
    max_altitude: float = 120.0        # m, for waypoint targets
    max_range: float = 500.0           # m from origin, for waypoint targets


def _finite(*vals: float) -> None:
    for v in vals:
        if not math.isfinite(v):
            raise ValueError("action contains a non-finite number")


class VelocityAction(BaseModel):
    """Velocity setpoint. `frame` = body (default, vehicle-relative) or world."""
    type: Literal["velocity"] = "velocity"
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    frame: Literal["body", "world"] = "body"

    @model_validator(mode="after")
    def _check(self):
        _finite(self.vx, self.vy, self.vz, self.yaw_rate)
        return self

    def clamp(self, lim: ActionLimits) -> "VelocityAction":
        h = math.hypot(self.vx, self.vy)
        s = lim.max_speed_xy / h if h > lim.max_speed_xy else 1.0
        vz = max(-lim.max_speed_z, min(lim.max_speed_z, self.vz))
        yr = max(-lim.max_yaw_rate, min(lim.max_yaw_rate, self.yaw_rate))
        return self.model_copy(update={"vx": self.vx * s, "vy": self.vy * s, "vz": vz, "yaw_rate": yr})

    def violations(self, lim: ActionLimits) -> list[str]:
        out = []
        if math.hypot(self.vx, self.vy) > lim.max_speed_xy + 1e-9:
            out.append(f"horizontal speed exceeds {lim.max_speed_xy} m/s")
        if abs(self.vz) > lim.max_speed_z + 1e-9:
            out.append(f"vertical speed exceeds {lim.max_speed_z} m/s")
        if abs(self.yaw_rate) > lim.max_yaw_rate + 1e-9:
            out.append(f"yaw rate exceeds {lim.max_yaw_rate} rad/s")
        return out


class WaypointAction(BaseModel):
    """Go to a world-frame position; the environment's own controller produces velocities."""
    type: Literal["waypoint"] = "waypoint"
    x: float
    y: float
    z: float
    yaw: float | None = None           # rad, world frame; None = keep heading
    speed: float = 2.0                 # m/s cruise limit
    tolerance: float = 0.3             # m, considered reached

    @model_validator(mode="after")
    def _check(self):
        _finite(self.x, self.y, self.z, self.speed, self.tolerance, self.yaw or 0.0)
        if self.speed <= 0:
            raise ValueError("speed must be > 0")
        return self

    def violations(self, lim: ActionLimits) -> list[str]:
        out = []
        if self.z > lim.max_altitude:
            out.append(f"altitude exceeds {lim.max_altitude} m")
        if math.hypot(self.x, self.y) > lim.max_range:
            out.append(f"target beyond {lim.max_range} m range")
        if self.speed > lim.max_speed_xy:
            out.append(f"speed exceeds {lim.max_speed_xy} m/s")
        return out


class HoldAction(BaseModel):
    """Hold position (zero velocity setpoint)."""
    type: Literal["hold"] = "hold"


class ArmAction(BaseModel):
    type: Literal["arm"] = "arm"
    armed: bool = True


Action = VelocityAction | WaypointAction | HoldAction | ArmAction


class ActionEnvelope(BaseModel):
    action: Action


class ActionSpace(BaseModel):
    """Description of what an agent accepts (for clients / RL wrappers)."""
    level: str                                  # "velocity" | "waypoint"
    types: list[str]
    fields: dict[str, Any]
    limits: ActionLimits
    held_between_steps: bool = True


class ObservationSpace(BaseModel):
    profile: str
    components: list[str]
    frames: list[dict[str, Any]] = Field(default_factory=list)   # camera descriptors


class AgentInfo(BaseModel):
    agent_id: str
    agent_type: str
    template: str
    observation_space: ObservationSpace
    action_space: ActionSpace
    sensors: list[str]
    control_mode: str
    state: EntityState | None = None


# ---------------------------------------------------------------------------
# episodes, stepping, events
# ---------------------------------------------------------------------------

EpisodeStatus = Literal["created", "running", "paused", "completed", "terminated", "failed"]


class Episode(BaseModel):
    episode_id: str
    scenario_id: str
    seed: int
    mode: SimMode
    start_time: str                              # wall clock ISO-8601
    sim_time: float = 0.0
    step_count: int = 0                          # API-level steps (stepped mode)
    iterations: int = 0                          # physics iterations
    status: EpisodeStatus = "created"
    max_sim_time: float | None = None
    log_path: str | None = None
    randomization: dict[str, Any] = Field(default_factory=dict)   # seeded initial-condition draw


class Event(BaseModel):
    seq: int
    event: str                                   # collision | landing | takeoff | out_of_bounds | timeout | ...
    sim_time: float
    entities: list[str] = Field(default_factory=list)
    data: dict[str, Any] = Field(default_factory=dict)
    episode_id: str | None = None


class ResetRequest(BaseModel):
    seed: int | None = None
    scenario: str | None = None
    mode: SimMode | None = None


class StepRequest(BaseModel):
    steps: int = 1
    actions: dict[str, Action] | None = None
    observe: bool = True

    @field_validator("steps")
    @classmethod
    def _steps(cls, v):
        if v < 1 or v > 100000:
            raise ValueError("steps must be in 1..100000")
        return v


class StepResponse(BaseModel):
    episode: Episode
    status: SimStatus
    observations: dict[str, Observation] = Field(default_factory=dict)
    events: list[Event] = Field(default_factory=list)
    rejected_actions: dict[str, str] = Field(default_factory=dict)


class ModeRequest(BaseModel):
    mode: SimMode


class SpawnRequest(BaseModel):
    entity_id: str
    kind: EntityKind = "drone"
    template: str = "quadcopter"
    pose: Pose = Field(default_factory=Pose)
    params: dict[str, Any] = Field(default_factory=dict)
    observation: str | None = None               # profile name for agents
    camera: dict[str, Any] | None = None         # legacy CameraSpec fields; prefer `sensors`
    drone_type: str = "standard"                 # standard | light | heavy
    control: Literal["velocity", "waypoint"] = "waypoint"   # highest action level the new agent accepts
    sensors: list[dict[str, Any]] | None = None  # SensorMount dicts (name, type, pose, params)
    trajectory: dict[str, Any] | None = None     # TrajectorySpec for non-agent entities (circle/line/waypoints/rotate)


class Metrics(BaseModel):
    real_time_factor: float = 0.0
    sim_hz: float = 0.0                          # physics iterations per wall second
    step_size: float | None = None
    sim_time: float = 0.0
    entity_count: int = 0
    agent_count: int = 0
    telemetry_clients: int = 0
    sensor_clients: int = 0
    api_step_latency_ms: float | None = None     # EMA of /simulation/step wall time
    sensor_fps: dict[str, float] = Field(default_factory=dict)
    events_total: int = 0
    uptime_s: float = 0.0


# ---------------------------------------------------------------------------
# scene description for the browser
# ---------------------------------------------------------------------------

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
    collisions: list[Visual] = Field(default_factory=list)


class ModelDesc(BaseModel):
    entity_id: str
    kind: EntityKind = "unknown"
    is_agent: bool = False
    category: str = "obstacle"
    pose: Pose = Field(default_factory=Pose)    # world frame
    links: list[LinkDesc] = Field(default_factory=list)
    is_static: bool = False


class SceneDesc(BaseModel):
    world: str
    models: list[ModelDesc]
    bounds: dict[str, list[float]] | None = None
    environment: dict[str, Any] | None = None   # lighting preset, sun, ambient, fog, wind (see sdf.environment_block)
    areas: list[dict[str, Any]] = Field(default_factory=list)   # named regions for the hierarchy / overview
