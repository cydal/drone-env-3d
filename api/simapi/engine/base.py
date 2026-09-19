"""Engine interface. Everything simulator-specific implements this."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable

from ..models import Action, EntityInfo, EntityState, ImuReading, Pose, SceneDesc, SimStatus
from ..scenario import Scenario


class SimulationEngine(ABC):
    @abstractmethod
    async def start(self, scenario: Scenario) -> None: ...

    @abstractmethod
    async def shutdown(self) -> None: ...

    @abstractmethod
    async def reset(self) -> None: ...

    @abstractmethod
    async def pause(self) -> None: ...

    @abstractmethod
    async def resume(self) -> None: ...

    @abstractmethod
    async def step(self, steps: int = 1) -> None: ...

    @abstractmethod
    def status(self) -> SimStatus: ...

    @abstractmethod
    async def scene(self) -> SceneDesc: ...

    @abstractmethod
    def list_entities(self) -> list[EntityInfo]: ...

    @abstractmethod
    def entity_state(self, entity_id: str) -> EntityState | None: ...

    @abstractmethod
    def imu(self, agent_id: str) -> ImuReading | None: ...

    @abstractmethod
    def sensor_names(self, agent_id: str) -> list[str]: ...

    @abstractmethod
    def sensor_frame(self, agent_id: str, sensor: str) -> tuple[bytes, dict] | None:
        """Latest frame as (raw bytes, metadata)."""

    @abstractmethod
    async def spawn(self, entity_id: str, template: str, pose: Pose, params: dict) -> None: ...

    @abstractmethod
    async def remove(self, entity_id: str) -> None: ...

    @abstractmethod
    async def send_action(self, agent_id: str, action: Action) -> None: ...

    @abstractmethod
    def on_pose_update(self, cb: Callable[[float, dict[str, Pose]], None]) -> None:
        """Register a callback fired from the engine thread with (sim_time, {entity_id: pose})."""
