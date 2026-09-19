"""Environment recording, snapshots and replay.

Design (see docs/ARCHITECTURE.md, "Snapshots"): gz-sim cannot restore a mid-episode
physical state through its state service (verified: physics keeps going), but the world
is deterministic given scenario + seed + the exact sequence of actions per iteration.
So:

* every applied action is logged with its iteration stamp (cheap, always on);
* a *snapshot* = captured entity states + a pointer into that action log;
* *restore* = reset(scenario, seed) and re-apply the log up to the pointer (exact in
  stepped mode; realtime-recorded actions are re-applied at their nearest iteration);
* *replay* = the same, to the end of the log, optionally paced for the browser;
* optional *rich rows* (per step: observations, states, events) form a replay buffer that
  external learners can pull incrementally or export as arrays.
"""
from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .models import Action, EntityState


@dataclass
class ActionRecord:
    """One logged event: an agent action (`action`), or a world operation (`op` in
    spawn | remove | teleport with `args`). Both carry the iteration they were applied at."""
    iteration: int
    sim_time: float
    agent_id: str | None = None
    action: dict[str, Any] | None = None
    op: str | None = None
    args: dict[str, Any] | None = None


@dataclass
class RecordingMeta:
    recording_id: str
    episode_id: str
    scenario: str
    seed: int
    mode: str
    step_size: float
    created: float
    observations: bool = False
    states: bool = True
    frames: bool = False
    rows: int = 0
    actions: int = 0
    final_iteration: int = 0
    status: str = "recording"          # recording | stopped


class Recorder:
    """One recorder per episode. Actions are always logged; rows are opt-in."""

    def __init__(self, directory: Path, *, episode_id: str, scenario: str, seed: int, mode: str, step_size: float) -> None:
        self.dir = directory / episode_id
        self.dir.mkdir(parents=True, exist_ok=True)
        self.meta = RecordingMeta(recording_id=episode_id, episode_id=episode_id, scenario=scenario, seed=seed, mode=mode,
                                  step_size=step_size, created=time.time())
        self.actions: list[ActionRecord] = []
        self._actions_f = (self.dir / "actions.jsonl").open("a", buffering=1)
        self._rows_f = None
        self.rows_enabled = False
        self._save_meta()

    # ---- actions (always) ----------------------------------------------------
    def log_action(self, iteration: int, sim_time: float, agent_id: str, action: Action) -> None:
        self._log(ActionRecord(iteration, sim_time, agent_id=agent_id, action=action.model_dump(mode="json")))

    def log_op(self, iteration: int, sim_time: float, op: str, args: dict[str, Any]) -> None:
        self._log(ActionRecord(iteration, sim_time, op=op, args=args))

    def _log(self, rec: ActionRecord) -> None:
        self.actions.append(rec)
        self._actions_f.write(json.dumps(rec.__dict__, separators=(",", ":"), default=_default) + "\n")
        self.meta.actions = len(self.actions)

    # ---- rows (opt-in replay buffer) ------------------------------------------
    def start_rows(self, *, observations: bool, states: bool, frames: bool) -> None:
        self.meta.observations, self.meta.states, self.meta.frames = observations, states, frames
        self._rows_f = (self.dir / "rows.jsonl").open("a", buffering=1)
        self.rows_enabled = True
        self._save_meta()

    def stop_rows(self) -> None:
        self.rows_enabled = False
        if self._rows_f:
            self._rows_f.close(); self._rows_f = None
        self.meta.status = "stopped"
        self._save_meta()

    def row(self, *, iteration: int, sim_time: float, actions: dict[str, Any], states: dict[str, Any],
            observations: dict[str, Any], events: list[dict[str, Any]]) -> None:
        if not self.rows_enabled or self._rows_f is None:
            return
        rec = {"i": self.meta.rows, "iteration": iteration, "sim_time": sim_time, "actions": actions,
               "states": states if self.meta.states else None,
               "observations": observations if self.meta.observations else None, "events": events}
        self._rows_f.write(json.dumps(rec, separators=(",", ":"), default=_default) + "\n")
        self.meta.rows += 1

    def read_rows(self, since: int = 0, limit: int = 1000) -> list[dict[str, Any]]:
        p = self.dir / "rows.jsonl"
        if not p.exists():
            return []
        out = []
        with p.open() as f:
            for k, line in enumerate(f):
                if k < since:
                    continue
                out.append(json.loads(line))
                if len(out) >= limit:
                    break
        return out

    def finalize(self, final_iteration: int) -> None:
        self.meta.final_iteration = final_iteration
        self.meta.status = "stopped"
        self.stop_rows()
        self._actions_f.close()
        self._save_meta()

    def _save_meta(self) -> None:
        (self.dir / "meta.json").write_text(json.dumps(self.meta.__dict__, indent=1))

    @staticmethod
    def load_actions(directory: Path) -> list[ActionRecord]:
        p = directory / "actions.jsonl"
        if not p.exists():
            return []
        return [ActionRecord(**json.loads(l)) for l in p.open() if l.strip()]

    @staticmethod
    def load_meta(directory: Path) -> RecordingMeta:
        return RecordingMeta(**json.loads((directory / "meta.json").read_text()))


# ---------------------------------------------------------------------------
# snapshots
# ---------------------------------------------------------------------------

@dataclass
class Snapshot:
    snapshot_id: str
    name: str
    created: float
    episode_id: str
    scenario: str
    seed: int
    mode: str
    sim_time: float
    iteration: int
    step_count: int
    recording_dir: str
    action_index: int                       # actions[:action_index] reproduce this state
    entities: dict[str, dict[str, Any]]     # entity_id -> EntityState
    agents: dict[str, dict[str, Any]]       # agent_id -> {control_mode, armed, last_action, waypoint}
    events_total: int = 0

    def to_json(self) -> dict[str, Any]:
        return self.__dict__


class SnapshotStore:
    def __init__(self, directory: Path) -> None:
        self.dir = directory
        self.dir.mkdir(parents=True, exist_ok=True)

    def save(self, snap: Snapshot) -> Path:
        p = self.dir / f"{snap.snapshot_id}.json"
        p.write_text(json.dumps(snap.to_json(), indent=1, default=_default))
        return p

    def load(self, snapshot_id: str) -> Snapshot:
        return Snapshot(**json.loads((self.dir / f"{snapshot_id}.json").read_text()))

    def list(self) -> list[dict[str, Any]]:
        out = []
        for p in sorted(self.dir.glob("*.json")):
            d = json.loads(p.read_text())
            out.append({k: d[k] for k in ("snapshot_id", "name", "created", "episode_id", "scenario", "seed", "sim_time", "iteration")})
        return out

    @staticmethod
    def new_id(name: str | None) -> str:
        stamp = time.strftime("%Y%m%d-%H%M%S")
        return f"{stamp}_{(name or 'snapshot').replace(' ', '_')}_{uuid.uuid4().hex[:4]}"


def _default(o):
    if hasattr(o, "model_dump"):
        return o.model_dump(mode="json")
    if hasattr(o, "__dict__"):
        return o.__dict__
    raise TypeError(str(type(o)))
