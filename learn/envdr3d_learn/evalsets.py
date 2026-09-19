"""Fixed, versioned evaluation sets. Generated once from a seed and saved as JSON so they never
change between experiments (brief 3 §15). Regenerating with the same seed reproduces them."""
from __future__ import annotations

import json
import random
from pathlib import Path
from typing import Any

from .task import LEVELS, Region

ROOT = Path(__file__).resolve().parents[2]
EVAL_DIR = ROOT / "eval_sets"


def generate(level: str, n: int, seed: int, *, start: Region | None = None, target: Region | None = None) -> list[dict[str, Any]]:
    rng = random.Random(seed)
    s_reg = start or LEVELS[level]["start"]
    t_reg = target or LEVELS[level]["target"]
    pairs = []
    for i in range(n):
        s = s_reg.sample(rng)
        t = t_reg.sample(rng)
        pairs.append({"id": f"{level}-{i:02d}", "start": [round(v, 2) for v in s], "target": [round(v, 2) for v in t]})
    return pairs


def save(level: str, name: str, pairs: list[dict[str, Any]], seed: int) -> Path:
    EVAL_DIR.mkdir(exist_ok=True)
    p = EVAL_DIR / f"{name}.json"
    p.write_text(json.dumps({"level": level, "seed": seed, "n": len(pairs), "pairs": pairs}, indent=1))
    return p


def load(name: str) -> dict[str, Any]:
    return json.loads((EVAL_DIR / f"{name}.json").read_text())


if __name__ == "__main__":
    # The canonical sets. Training uses *different* seeds/regions (random per episode), so
    # these start/target pairs are unseen during training.
    for level, name, seed in (("open", "level1_open_eval", 9001), ("pillars", "level2_pillars_eval", 9002), ("city", "level3_city_eval", 9003)):
        p = save(level, name, generate(level, 20, seed), seed)
        print("wrote", p)
