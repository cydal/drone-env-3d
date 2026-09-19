# env-dr3d — autonomous simulation world

A general-purpose physical simulation environment (**Gazebo Sim Jetty**, headless)
with a browser interface and an external-agent API. The simulator is authoritative;
the browser only visualizes, and external Python controllers only talk to the API
— nobody touches Gazebo directly except the API's engine adapter.

See [`briefs/brief-1.md`](briefs/brief-1.md) for the original goals and
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the design and the decisions
behind it (why Jetty, why this control abstraction, stepping/determinism model).

```
sim/       worlds (SDF), scenarios (YAML), model templates
api/       Simulation API server (FastAPI + WebSocket) and the Gazebo engine adapter
client/    Python SDK for external controllers (HTTP only, no Gazebo knowledge)
web/       browser frontend (Vite + three.js)
examples/  external controllers (integration tests)
runs/      per-run artefacts: scenario.yaml, generated world.sdf, gz-server.log
tests/     API contract tests (run without Gazebo, using a fake engine)
```

---

## Prerequisites

- macOS on Apple Silicon (this has only been run on macOS 26 "Tahoe", arm64).
  There's no Linux/Docker path here — see `docs/SENSORS.md` for why that will
  matter once camera sensors are added.
- [Homebrew](https://brew.sh)
- [uv](https://docs.astral.sh/uv/) for the Python venv (`brew install uv` if you
  don't have it)
- Node.js + npm (`brew install node` if you don't have it)

## First-time setup

Run these once, in order, from the repo root.

**1. Install Gazebo Sim Jetty** (gz-sim 10) via the Open Robotics tap:

```bash
brew tap osrf/simulation
brew trust osrf/simulation
brew install osrf/simulation/gz-jetty
```

This pulls in Qt, OGRE, DART and ~35 other packages — expect several minutes on
first install. Verify it worked:

```bash
gz sim --versions        # should print 10.x.x
```

**2. Create the Python virtual environment.**

Gazebo's Python bindings (`gz.transport`, `gz.msgs`) are built by Homebrew
against its own `python@3.14`, so the venv must be created from that same
interpreter with access to its site-packages:

```bash
uv venv --python /opt/homebrew/opt/python@3.14/bin/python3.14 --system-site-packages .venv
uv pip install --python .venv/bin/python -e ".[dev]"
```

`pyproject.toml` pins `protobuf>=7.36,<8` to match the protobuf compiler
Homebrew used to generate the `gz.msgs` bindings. If you ever see an error like
`Detected incompatible Protobuf Gencode/Runtime versions`, it means something
downgraded protobuf in the venv — reinstall with the command above.

**3. Install the web frontend's dependencies:**

```bash
cd web && npm install && cd ..
```

**4. Sanity-check the whole toolchain** (optional but recommended — confirms
Gazebo, the Python bindings and the API all agree with each other):

```bash
.venv/bin/python -m pytest -q tests   # API contract tests, no Gazebo needed
```

---

## Starting the server

### Quick way

```bash
scripts/dev.sh
```

This starts the web dev server (http://127.0.0.1:5173) in the background and
the Simulation API (http://127.0.0.1:8000) in the foreground. Gazebo itself is
**not** started yet at this point — the API launches `gz sim -s` headless only
once a scenario is loaded (see "Using it" below). Leave this terminal open;
`Ctrl-C` stops the API (and, via its shutdown hook, any running Gazebo server).

### Manual way (if you want the two halves in separate terminals)

```bash
# terminal 1 — API + Gazebo
source .venv/bin/activate
export GZ_PARTITION=envdr3d
PYTHONPATH=api python -m simapi

# terminal 2 — web UI
cd web && npm run dev -- --host 127.0.0.1
```

Then open **http://127.0.0.1:5173** in a browser, and the OpenAPI docs at
**http://127.0.0.1:8000/docs**.

## Using it

1. In the browser's top bar, pick a scenario (only `test_city_two_drones` exists
   so far) and click **Load** — or equivalently:
   ```bash
   curl -X POST localhost:8000/simulation/load/test_city_two_drones
   ```
   This generates `runs/<timestamp>_<scenario>/world.sdf`, launches
   `gz sim -s -r` headless against it, and the browser starts rendering as soon
   as the scene is available.
2. Use the transport controls (▶ pause/resume, ⏭ step, ↺ reset) or drive a
   drone from outside the browser with the example controller:
   ```bash
   .venv/bin/python examples/waypoint_controller.py --agent drone_01 --waypoint 8,4,6
   ```
   This is the Phase-1D integration test: take off → fly to waypoint → hover →
   land, driven entirely through the HTTP API (`client/simclient`), with no
   knowledge of Gazebo.

## Stopping everything cleanly

`Ctrl-C` in the `scripts/dev.sh` terminal stops the API and its Gazebo child.
If something was left running in the background (e.g. after a crashed
terminal), clean up with:

```bash
pkill -f "simapi"          # matches the API process regardless of which Python binary ran it
pkill -f "gz-sim-main"
pkill -f "npm run dev"
```

(Homebrew's `python@3.14` binary is named `Python`, capitalized, so a pattern
like `"python -m simapi"` can silently fail to match — `"simapi"` alone is
robust to that.)

Then confirm nothing is still listening:

```bash
lsof -nP -iTCP:8000 -iTCP:5173 -sTCP:LISTEN
```

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `Detected incompatible Protobuf Gencode/Runtime versions` | venv's `protobuf` package doesn't match Homebrew's protoc | `uv pip install --python .venv/bin/python "protobuf>=7.36,<8"` |
| `ModuleNotFoundError: No module named 'gz'` | venv wasn't created with `--system-site-packages` against Homebrew's python@3.14 | recreate the venv per step 2 above |
| API hangs on `/simulation/load/...` | a previous `gz-sim-main` from an earlier run is still alive and holding the transport partition | `pkill -f gz-sim-main`, wait a couple seconds, retry |
| `pkill -f "python -m simapi"` doesn't kill anything | Homebrew's python@3.14 binary is capitalized (`Python`), so a lowercase pattern won't match | use `pkill -f "simapi"` instead (case-insensitive to the binary name) |
| `address already in use` on port 8000/5173 | a previous server process wasn't stopped | run the cleanup commands above, then re-check with `lsof` |
| Browser shows nothing after Load | scenario hasn't finished spawning yet (first `gz sim` launch after a fresh install can take ~20s; later launches are ~3s) | wait a few seconds, check `runs/<latest>/gz-server.log` for errors |
| `status.running` is false, episode `failed`, event `simulator_crashed` | the Gazebo process died (its stack trace is in `runs/<latest>/gz-server.log`) | `POST /episode/reset` (or the ↺ button) relaunches the scenario automatically |
| Reset with cameras attached takes ~3 s | Gazebo re-initialises the rendering system on every world rewind | expected; use a camera-less scenario for fast RL loops or batch resets |

## Phase 2 — the environment loop

Phase 2 turns the platform into an *environment*: observations in, actions out,
episodes, events, two time modes. Full contract: [`docs/API.md`](docs/API.md).

```python
from simclient import Simulation            # client/ — HTTP only, no Gazebo knowledge

sim = Simulation("localhost")
episode = sim.reset(scenario="phase2_city", seed=42, mode="stepped")
obs = sim.observe("drone_01")
while not episode.done:
    action = controller(obs)                # e.g. sim.velocity(vx=1.0, vz=0.5) or sim.waypoint(8, 4, 6)
    result = sim.step(agent="drone_01", action=action, steps=25)   # exactly 25 physics iterations
    obs, episode = result.observations["drone_01"], result.episode
    for e in result.events: ...             # collision / landing / takeoff / out_of_bounds / timeout
```

Key ideas:

* **Observation profiles** decide what an agent may see (`state`, `navigation`,
  `vision`, `minimal`, `shared`, `central`, or scenario-defined). Ground truth
  stays available separately at `/entities/{id}`.
* **Actions** are validated (`velocity`, `waypoint`, `hold`, `arm`; limits per
  agent); bad actions return 422 and never reach the simulator.
* **Modes**: `realtime` (free-running, for humans) and `stepped` (advances only
  on `step`, for RL/planning/datasets).
* **Episodes** carry `episode_id / scenario / seed / mode / status`; every
  episode logs `runs/<run>/episode_<id>.jsonl` (state, action, events per step).
* **Events** are raw environment facts, not rewards.
* **Dynamic entities** (a circling target, a patrolling vehicle) move on their
  own along deterministic trajectories declared in the scenario.
* **Sensors**: RGB + depth frames over HTTP or a binary WebSocket, the same
  frames the browser shows.

### Browser

Realtime/Stepped toggle, play/pause/step/reset (seed box), entity list,
telemetry with observation profile and control mode, events log with collision
markers, Orbit/Follow/Top/FPV cameras, overlays (trails, velocity vectors, IDs,
axes, camera frustum, collision geometry, bounds), RGB/depth sensor panel, a
health strip (`/metrics`) and **manual keyboard flight** that goes through the
same public action API as any Python controller (W/S/A/D, R/F, Q/E, X hold,
Z arm/disarm; tick *Manual control* on a selected agent).

The camera auto-frames the agent cluster on load/reset instead of a wide,
mostly-empty establishing shot, agent labels/markers keep a constant on-screen
size regardless of distance so a drone never shrinks to an invisible speck,
and a **Fit view** button recentres on all agents at any time (useful after
one flies out of frame — there is no off-screen indicator yet, see Known limits).

### Example controllers (`examples/`)

```bash
.venv/bin/python examples/hover.py                 # climb + hold (realtime)
.venv/bin/python examples/waypoint_controller.py   # external P controller, stepped
.venv/bin/python examples/circle.py                # track a circular reference, stepped
.venv/bin/python examples/formation.py             # three drones in a triangle, stepped
.venv/bin/python examples/success_test.py          # brief 2 §29 checklist end to end
```

### Tests

```bash
.venv/bin/python -m pytest -q tests/test_api_fake_engine.py   # API contract, no Gazebo (fast)
.venv/bin/python -m pytest -q tests/integration                # real Gazebo: lifecycle, agents,
                                                               # reproducibility, collisions, frames (~3 min)
```

## Status (2026-09-19) — vertical slice verified

Gazebo → API → external Python controller → drone moves → browser sees it: **working.**

* Headless `gz sim -s` (Jetty 10.5.0) launched from a scenario; RTF ≈ 1.0 with two quadcopters.
* `examples/waypoint_controller.py` passes for `drone_01` and `drone_02` simultaneously
  (take off → waypoint → hover → land within ~5 cm).
* Pause / resume / blocking `step(N)` / reset via REST and WebSocket; three
  `reset → action → step(500)` runs give bit-identical positions.
* Runtime spawn/remove of extra drones (`POST /entities`, `DELETE /entities/{id}`).
* Browser: 3D world from the scene service, 25 Hz pose stream, entity list,
  telemetry panel (pose, world velocity, attitude, IMU), orbit/follow/top cameras.
* Every run writes `runs/<stamp>_<scenario>/{scenario.yaml,world.sdf,gz-server.log}`.

Not yet: recording/replay beyond the per-episode JSONL log, the nicer city, and an
on-screen indicator for agents that fly outside the current view (mitigated by the
Fit view button, see the browser section above).

## Tests

```bash
.venv/bin/python -m pytest -q tests          # API contract tests with a fake engine (no Gazebo needed)
```
