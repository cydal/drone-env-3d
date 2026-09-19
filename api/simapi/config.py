from __future__ import annotations

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SIM_DIR = ROOT / "sim"
WORLDS_DIR = SIM_DIR / "worlds"
MODELS_DIR = SIM_DIR / "models"
SCENARIOS_DIR = SIM_DIR / "scenarios"
RUNS_DIR = ROOT / "runs"

GZ_BIN = os.environ.get("GZ_BIN", "gz")
API_HOST = os.environ.get("SIMAPI_HOST", "127.0.0.1")
API_PORT = int(os.environ.get("SIMAPI_PORT", "8000"))
TELEMETRY_HZ = float(os.environ.get("SIMAPI_TELEMETRY_HZ", "30"))
