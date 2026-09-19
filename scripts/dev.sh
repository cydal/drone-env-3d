#!/usr/bin/env bash
# Start the Simulation API (headless Gazebo is launched on demand) and the web UI.
set -euo pipefail
cd "$(dirname "$0")/.."
export GZ_PARTITION="${GZ_PARTITION:-envdr3d}"
source .venv/bin/activate
(cd web && npm run dev -- --host 127.0.0.1 >/tmp/envdr3d-web.log 2>&1 &) 
echo "web:  http://127.0.0.1:5173"
echo "api:  http://127.0.0.1:8000/docs"
PYTHONPATH=api python -m simapi
