# Architecture

```
 browser (web/)         external agents (client/ SDK, examples/)
      │  WS + HTTP              │  HTTP + WS
      └───────────┬─────────────┘
            Simulation API (api/simapi)  ── FastAPI + WebSocket
                  │  engine interface (simapi.engine.base.SimulationEngine)
            GazeboEngine (simapi.engine.gazebo)  ── gz-transport (Python bindings)
                  │
            gz sim -s  (Gazebo Sim server, headless, authoritative state)
                  │
            sim/worlds/*.sdf   sim/models/*   sim/scenarios/*.yaml
```

## Decisions (2026-09-19)

* **Gazebo release: Jetty (gz-sim 10, LTS to May 2031).** Installed via the
  Open Robotics Homebrew tap (`brew install osrf/simulation/gz-jetty`).
  Modern `gz` CLI and `gz::sim::systems::*` plugins only. Nothing from
  Gazebo Classic (`gzserver`, `libgazebo_*.so`, `gazebo_ros`) is used.
* **Python bindings:** Jetty dropped the version suffix, so the modules are
  `gz.transport`, `gz.msgs`, `gz.math`, `gz.sim` (Ionic was `gz.transport14`,
  `gz.msgs11`). Homebrew builds them for its own `python@3.12/3.13/3.14`,
  so the API server venv is created from Homebrew Python with
  `--system-site-packages`.
* **Gazebo is the only source of truth.** The browser renders what the API
  streams; it never simulates.
* **Engine is replaceable.** Everything Gazebo-specific lives behind
  `SimulationEngine`. The REST/WS schema, scenario format and client SDK
  have no Gazebo concepts in them.
* **Agents are plural from day one.** Every agent has an `agent_id`, its
  own namespace inside Gazebo (`/model/<id>/...`), its own observation and
  action interface.
* **Drone control abstraction (Phase 1):** body-frame velocity + yaw-rate
  setpoints handled by Gazebo's `MulticopterVelocityControl` system, which
  drives four `MulticopterMotorModel` rotors. This is a real rigid body with
  thrust/torque per rotor; lower-level (motor speed) control can be exposed
  later by publishing `gz.msgs.Actuators` directly.
* **macOS notes:** server (`gz sim -s`) and GUI must be separate processes;
  we don't use the GUI at all. Camera/depth sensors need the Sensors system
  with a render engine; headless rendering via EGL is Linux-only, so on macOS
  image sensors are validated with a display attached (see docs/SENSORS.md).

## Vertical slice (first milestone, done)

Gazebo → API → external Python controller → drone moves → browser sees it.

## Phase 2 layering

```
simapi.app          HTTP/WS routes only; maps errors to 409/422
simapi.service      the *environment*: episodes, modes, observation profiles,
                    action validation, waypoint controller, trajectory controller,
                    events, JSONL logging, metrics
simapi.engine.base  what a simulator must provide (start/reset/step, poses,
                    sensors, set_pose, send_velocity, callbacks)
simapi.engine.gazebo Gazebo Jetty adapter (gz-transport, SDF generation, process)
simclient           external client: Simulation / Episode / Observation / StepResult
web/                browser: same public operations, plus rendering & debug overlays
```

Environment-owned intelligence is limited to two deterministic controllers:
the waypoint P controller (agent level) and the trajectory controller
(environment entities). Both run inside `step()` in stepped mode and on a 20 Hz
tick in realtime mode; the tick is inert in stepped mode so nothing races a step.

See `docs/API.md` for the agent contract, observation profiles, action space,
stepping semantics, events and transports.

## Stepping and determinism

* `POST /simulation/step {steps: N, actions}` blocks until Gazebo has advanced
  exactly N iterations and is paused again. Long steps are sub-stepped (25
  iterations) when environment entities move, so they travel continuously.
* Verified: identical `reset → act → step(401)` sequences reproduce positions to
  7 decimals; resting contact jitter between resets is ~1e-7 m (DART floor).
* **Service requests run in a separate process** (`engine/gazebo/reqworker.py`).
  The gz-transport Python binding's `request()` blocks while holding the GIL,
  and gz-transport's receive thread needs the GIL to deliver subscription
  callbacks, so under the API's callback load every in-process request timed
  out and froze the interpreter (measured 15/15). A worker process with its own
  Node and no subscriptions answers in ~1 ms. Publishing stays in-process (it
  never waits). Wall-clock throttling of subscriptions is deliberately *not*
  used: observation freshness must depend on sim time only.
* Poses are published every physics iteration (`dynamic_pose_hertz` ≥ 1/step)
  and `step()` waits for the pose stamped with the final sim time, so the state
  read after a step is exact, not "latest within a few ms".
* Known transport hazards and their mitigations (all bit us during development):
  - gz-transport drops messages published before discovery connects a freshly
    (re)created subscriber → command publishers are advertised at agent attach,
    a 0.4 s settle follows start/reset, and each agent's latched command is
    re-sent every step.
  - Gazebo reports *unpaused* while executing a multi-step → the realtime tick
    is disabled in stepped mode; the desired pause state is tracked, not observed.
  - A world reset is asynchronous → `reset` waits for the rewind to be observed
    in world stats, retries once, and fails loudly instead of continuing.
  - After a reset with cameras attached the server stalls ~3 s re-initialising
    rendering → control requests use an 8 s timeout.
* `reset` uses `WorldControl.reset.all`; Gazebo re-creates the systems, so the
  multicopter controllers idle until the next command. A different seed
  relaunches the server (`--seed` is a launch parameter); physics itself is
  deterministic, so the seed matters only for future noise/randomised scenarios.
* A strictly lock-stepped path (actions applied *inside* the physics step via a
  custom gz system) remains the Phase 3 upgrade if sub-millisecond action
  timing ever matters.
