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

## Vertical slice (first milestone)

Gazebo → API → external Python controller → drone moves → browser sees it.

## Stepping and determinism (current state)

* `POST /simulation/step {steps: N}` blocks until Gazebo has advanced exactly N
  iterations and is paused again. With a fixed `step_size`, seed and scenario,
  repeated `reset → actions → step(N)` sequences reproduce identical states.
* Actions travel over gz-transport asynchronously, so `send_action` followed by
  `step` has a tiny race window; the engine waits a few ms before stepping.
  A strictly lock-stepped path (actions applied inside the step) is a Phase 2
  item — candidates are `/world/<w>/control/state` (ECM state piggybacked on
  the control message) or a small custom gz system.
* `reset` uses `WorldControl.reset.all`; Gazebo re-creates the systems, so the
  multicopter controllers idle until the next command.
