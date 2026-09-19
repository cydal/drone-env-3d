"""Scenario = reproducible description of a simulation run (YAML)."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field

from .models import EntityKind, Pose, Vec3


class AgentSpec(BaseModel):
    id: str
    type: EntityKind = "drone"
    template: str = "quadcopter"
    spawn: Pose = Field(default_factory=Pose)
    sensors: dict[str, Any] = Field(default_factory=dict)
    params: dict[str, Any] = Field(default_factory=dict)


class WorldSpec(BaseModel):
    name: str                      # world name inside the SDF
    file: str                      # path relative to sim/worlds


class EnvironmentSpec(BaseModel):
    wind: Vec3 = Field(default_factory=Vec3)
    time_of_day: float | None = None
    visibility: float | None = None


class SimulationSpec(BaseModel):
    step_size: float = 0.004       # seconds per physics step
    real_time_factor: float = 1.0
    seed: int = 0
    start_paused: bool = True


class Scenario(BaseModel):
    name: str
    description: str = ""
    world: WorldSpec
    agents: list[AgentSpec] = Field(default_factory=list)
    environment: EnvironmentSpec = Field(default_factory=EnvironmentSpec)
    simulation: SimulationSpec = Field(default_factory=SimulationSpec)

    @classmethod
    def load(cls, path: Path) -> "Scenario":
        return cls.model_validate(yaml.safe_load(path.read_text()))

    def save(self, path: Path) -> None:
        path.write_text(yaml.safe_dump(self.model_dump(mode="json"), sort_keys=False))


def list_scenarios(directory: Path) -> list[str]:
    return sorted(p.stem for p in directory.glob("*.yaml"))
