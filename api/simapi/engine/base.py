"""Engine interface. Everything simulator-specific implements this.

Thread model: methods are called from the asyncio loop; the two callbacks
(`on_pose_update`, `on_event`) may fire from engine threads.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Callable

from ..models import (EntityInfo, EntityState, GpsReading, ImuReading, Pose, SceneDesc, SimStatus,
                      VelocityAction)
from ..scenario import Scenario

# raw engine events: (name, sim_time, entities, data)
EngineEvent = tuple[str, float, list[str], dict[str, Any]]


class RawFrame:
    """Latest frame of an image sensor, undecoded."""
    __slots__ = ("data", "width", "height", "pixel_format", "sim_time", "seq", "wall_time")

    def __init__(self, data: bytes, width: int, height: int, pixel_format: str,
                 sim_time: float, seq: int, wall_time: float) -> None:
        self.data, self.width, self.height = data, width, height
        self.pixel_format, self.sim_time, self.seq, self.wall_time = pixel_format, sim_time, seq, wall_time


class SimulationEngine(ABC):
    # ---- lifecycle
    @abstractmethod
    async def start(self, scenario: Scenario, *, seed: int | None = None) -> None: ...

    @abstractmethod
    async def shutdown(self) -> None: ...

    @abstractmethod
    async def reset(self) -> None:
        """Rewind to the initial state of the loaded scenario (same seed)."""

    @abstractmethod
    async def pause(self) -> None: ...

    @abstractmethod
    async def resume(self) -> None: ...

    @abstractmethod
    async def step(self, steps: int = 1) -> None:
        """Advance exactly `steps` physics iterations while paused; blocks until done."""

    @abstractmethod
    def status(self) -> SimStatus: ...

    @property
    @abstractmethod
    def seed(self) -> int: ...

    # ---- world / entities
    @abstractmethod
    async def scene(self) -> SceneDesc: ...

    @abstractmethod
    def list_entities(self) -> list[EntityInfo]: ...

    @abstractmethod
    def entity_state(self, entity_id: str) -> EntityState | None: ...

    @abstractmethod
    async def spawn(self, entity_id: str, template: str, pose: Pose, params: dict,
                    *, camera=None, drone_type: str = "standard", mounts=None) -> bool:
        """Returns True if the new entity is an agent."""

    @abstractmethod
    async def remove(self, entity_id: str) -> None: ...

    @abstractmethod
    def set_pose(self, entity_id: str, pose: Pose) -> None:
        """Kinematically place an entity (used for environment-driven objects)."""

    # ---- agent sensors
    @abstractmethod
    def agent_ids(self) -> list[str]: ...

    @abstractmethod
    def imu(self, agent_id: str) -> ImuReading | None: ...

    @abstractmethod
    def gps(self, agent_id: str) -> GpsReading | None: ...

    @abstractmethod
    def body_velocity(self, agent_id: str) -> tuple[Any, Any] | None:
        """(linear Vec3, angular Vec3) in the body frame."""

    @abstractmethod
    def grounded(self, agent_id: str) -> bool | None: ...

    @abstractmethod
    def sensor_names(self, agent_id: str) -> list[str]: ...

    def sensor_mounts(self, agent_id: str) -> list:
        return []

    def set_speed(self, real_time_factor: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def frame(self, agent_id: str, sensor: str) -> RawFrame | None: ...

    @abstractmethod
    def sensor_rate(self, agent_id: str, sensor: str) -> float: ...

    # ---- actions (engine-level: only what the simulator executes directly)
    @abstractmethod
    def send_velocity(self, agent_id: str, action: VelocityAction) -> None: ...

    @abstractmethod
    def send_arm(self, agent_id: str, armed: bool) -> None: ...

    # ---- callbacks
    @abstractmethod
    def on_pose_update(self, cb: Callable[[float, dict[str, Pose]], None]) -> None: ...

    @abstractmethod
    def on_event(self, cb: Callable[[EngineEvent], None]) -> None: ...
