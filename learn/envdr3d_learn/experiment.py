"""Experiment tracking: every run gets a directory with everything needed to reproduce it."""
from __future__ import annotations

import datetime as dt
import json
import platform
import subprocess
import sys
import uuid
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
EXPERIMENTS = ROOT / "experiments"


def _jsonable(o: Any) -> Any:
    if is_dataclass(o):
        return asdict(o)
    if isinstance(o, Path):
        return str(o)
    if isinstance(o, tuple):
        return list(o)
    raise TypeError(str(type(o)))


class Experiment:
    def __init__(self, name: str, config: dict[str, Any], *, root: Path = EXPERIMENTS) -> None:
        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.id = f"{stamp}_{name}_{uuid.uuid4().hex[:4]}"
        self.dir = root / self.id
        self.dir.mkdir(parents=True, exist_ok=True)
        try:
            commit = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, text=True).strip()
        except Exception:
            commit = None
        self.meta = {"experiment_id": self.id, "name": name, "created": dt.datetime.now().isoformat(timespec="seconds"),
                     "git_commit": commit, "python": sys.version.split()[0], "platform": platform.platform(),
                     "config": config, "results": {}}
        self.save()

    def save(self) -> None:
        (self.dir / "experiment.json").write_text(json.dumps(self.meta, indent=2, default=_jsonable))

    def log_result(self, key: str, value: Any) -> None:
        self.meta["results"][key] = value
        self.save()

    def path(self, *parts: str) -> Path:
        p = self.dir.joinpath(*parts)
        p.parent.mkdir(parents=True, exist_ok=True)
        return p


def load_experiment(exp_id_or_dir: str) -> dict[str, Any]:
    p = Path(exp_id_or_dir)
    if not p.exists():
        p = EXPERIMENTS / exp_id_or_dir
    return json.loads((p / "experiment.json").read_text())
