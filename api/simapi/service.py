"""SimulationService: owns the engine, the loaded scenario and telemetry."""
from __future__ import annotations

import asyncio
import logging
import time
from pathlib import Path

from . import config
from .engine.base import SimulationEngine
from .hub import TelemetryHub
from .models import (Action, EntityInfo, EntityState, Observation, Pose,
                     SceneDesc, SimStatus)
from .scenario import Scenario, list_scenarios

log = logging.getLogger(__name__)


class SimulationService:
    def __init__(self, engine: SimulationEngine, hub: TelemetryHub) -> None:
        self.engine = engine
        self.hub = hub
        self.scenario: Scenario | None = None
        self._last_broadcast = 0.0
        self._min_interval = 1.0 / config.TELEMETRY_HZ
        self.engine.on_pose_update(self._on_pose_update)

    # ---- lifecycle --------------------------------------------------------
    def scenarios(self) -> list[str]:
        return list_scenarios(config.SCENARIOS_DIR)

    def scenario_path(self, name: str) -> Path:
        return config.SCENARIOS_DIR / f"{name}.yaml"

    async def load_scenario(self, name: str) -> SimStatus:
        scenario = Scenario.load(self.scenario_path(name))
        return await self.start(scenario)

    async def start(self, scenario: Scenario) -> SimStatus:
        if self.scenario is not None:
            await self.engine.shutdown()
        self.scenario = scenario
        await self.engine.start(scenario)
        self.hub.publish_threadsafe({"type": "scenario_loaded", "scenario": scenario.model_dump(mode="json")})
        return self.status()

    async def shutdown(self) -> None:
        await self.engine.shutdown()
        self.scenario = None

    async def reset(self) -> SimStatus:
        await self.engine.reset()
        self.hub.publish_threadsafe({"type": "reset"})
        return self.status()

    async def pause(self) -> SimStatus:
        await self.engine.pause()
        return self.status()

    async def resume(self) -> SimStatus:
        await self.engine.resume()
        return self.status()

    async def step(self, steps: int = 1) -> SimStatus:
        await self.engine.step(steps)
        return self.status()

    def status(self) -> SimStatus:
        s = self.engine.status()
        if self.scenario:
            s.scenario = self.scenario.name
            s.seed = self.scenario.simulation.seed
            s.step_size = self.scenario.simulation.step_size
        return s

    # ---- world / entities -------------------------------------------------
    async def scene(self) -> SceneDesc:
        return await self.engine.scene()

    def entities(self) -> list[EntityInfo]:
        return self.engine.list_entities()

    def entity_state(self, entity_id: str) -> EntityState | None:
        return self.engine.entity_state(entity_id)

    async def spawn(self, entity_id: str, template: str, pose: Pose, params: dict) -> None:
        await self.engine.spawn(entity_id, template, pose, params)
        self.hub.publish_threadsafe({"type": "scene_changed"})

    async def remove(self, entity_id: str) -> None:
        await self.engine.remove(entity_id)
        self.hub.publish_threadsafe({"type": "scene_changed"})

    # ---- agents -----------------------------------------------------------
    def agent_ids(self) -> list[str]:
        return [e.entity_id for e in self.engine.list_entities() if e.is_agent]

    def observation(self, agent_id: str) -> Observation | None:
        state = self.engine.entity_state(agent_id)
        if state is None:
            return None
        return Observation(
            agent_id=agent_id,
            sim_time=state.sim_time,
            state=state,
            imu=self.engine.imu(agent_id),
            sensors=self.engine.sensor_names(agent_id),
        )

    def sensor_frame(self, agent_id: str, sensor: str):
        return self.engine.sensor_frame(agent_id, sensor)

    async def send_action(self, agent_id: str, action: Action) -> None:
        await self.engine.send_action(agent_id, action)

    # ---- telemetry --------------------------------------------------------
    def _on_pose_update(self, sim_time: float, poses: dict[str, Pose]) -> None:
        now = time.monotonic()
        if now - self._last_broadcast < self._min_interval:
            return
        self._last_broadcast = now
        st = self.engine.status()
        self.hub.publish_threadsafe({
            "type": "state",
            "sim_time": sim_time,
            "paused": st.paused,
            "rtf": st.real_time_factor,
            "iterations": st.iterations,
            "poses": {k: _pose_compact(v) for k, v in poses.items()},
        })


def _pose_compact(p: Pose) -> list[float]:
    return [p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
