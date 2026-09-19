"""Structured per-episode logging (JSONL). Foundation for later dataset generation."""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


class EpisodeLogger:
    def __init__(self, path: Path, header: dict[str, Any]) -> None:
        self.path = path
        self._f = path.open("a", buffering=1)
        self.write({"type": "episode", **header})

    def write(self, rec: dict[str, Any]) -> None:
        rec.setdefault("t_wall", time.time())
        self._f.write(json.dumps(rec, separators=(",", ":"), default=_default) + "\n")

    def sample(self, *, sim_time: float, iteration: int, step: int, agents: dict[str, Any],
               events: list[dict[str, Any]]) -> None:
        self.write({"type": "step", "sim_time": sim_time, "iteration": iteration, "step": step,
                    "agents": agents, "events": events})

    def event(self, ev: dict[str, Any]) -> None:
        self.write({"type": "event", **ev})

    def close(self, status: str) -> None:
        try:
            self.write({"type": "end", "status": status})
            self._f.close()
        except Exception:
            pass


def _default(o):
    if hasattr(o, "model_dump"):
        return o.model_dump(mode="json")
    raise TypeError(f"not serialisable: {type(o)}")
