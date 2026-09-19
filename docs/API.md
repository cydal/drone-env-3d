# Simulation API — agent interface (Phase 2)

Base URL `http://127.0.0.1:8000`. OpenAPI: `/docs`. Everything an external
controller or the browser can do goes through this surface; there is no side
channel to Gazebo.

## The contract

```
observation = observe(agent_id)        GET  /agents/{id}/observation
act(agent_id, action)                  POST /agents/{id}/action
step(actions, n)  -> observations      POST /simulation/step
reset(seed, scenario) -> observations  POST /episode/reset
```

Every agent exposes:

| field | source |
|---|---|
| `agent_id`, `agent_type`, `template` | scenario `agents[]` |
| `observation_space` | profile name + components + camera descriptors |
| `action_space` | level (`velocity` or `waypoint`), accepted types, limits |
| `sensors` | what is physically attached (imu, navsat, contact, camera, depth) |
| `control_mode` | `idle` / `velocity` / `hold` / `waypoint` |

`GET /agents` lists them; `GET /agents/{id}` gives one including ground-truth
`state` (privileged; for tooling, not policies).

## Ground truth vs observation

* `GET /entities/{id}` — privileged simulator state for **any** entity: pose,
  world-frame linear velocity, body-frame angular velocity, acceleration.
* `GET /agents/{id}/observation` — only what the agent's **profile** allows.

### Observation profiles

| profile | components |
|---|---|
| `state` | `state` (privileged pose/velocities of the agent itself) |
| `navigation` | `gps`, `imu`, `velocity` (body frame) |
| `vision` | `camera`, `depth`, `imu` |
| `minimal` | `camera`, `imu` |
| `shared` | `state`, `nearby_agents` (relative positions within 30 m) |
| `central` | `state`, `swarm_state` (ground truth of every agent) |
| `full` | everything |

Scenarios add their own under `observation_profiles:` and select one per agent
with `observation:`. Components: `state velocity imu gps camera depth
nearby_agents swarm_state`.

### Observation payload

```jsonc
{
  "agent_id": "drone_02", "sim_time": 5.9, "iteration": 1476, "profile": "navigation",
  "velocity": {"linear": {...}, "angular": {...}},        // body frame
  "imu": {"linear_acceleration": {...}, "angular_velocity": {...}, "orientation": {...}},
  "gps": {"latitude_deg": 47.39, "longitude_deg": 8.54, "altitude": 488.1, "velocity_enu": {...}},
  "frames": [ {"name": "camera", "type": "rgb", "width": 320, "height": 240, "seq": 67,
               "url": "/agents/drone_01/sensors/camera", "stream": "/ws/sensors/drone_01/camera"} ],
  "grounded": true, "armed": true, "control_mode": "idle"
}
```

Image frames are never inlined in JSON (see *Sensor transport*).

## Actions

```jsonc
{"type": "velocity", "vx": 1.0, "vy": 0.0, "vz": 0.5, "yaw_rate": 0.0, "frame": "body"}   // m/s, rad/s; frame body|world
{"type": "waypoint", "x": 8, "y": 4, "z": 6, "yaw": null, "speed": 2.0, "tolerance": 0.3}  // world frame
{"type": "hold"}                                                                           // zero velocity setpoint
{"type": "arm", "armed": false}                                                            // motors off -> falls
```

Levels: an agent with `control: velocity` accepts velocity/hold/arm; with
`control: waypoint` it also accepts waypoints, which an **environment-side**
P controller turns into velocity setpoints each tick/step. Velocity setpoints
are executed by Gazebo's `MulticopterVelocityControl` (a real rigid body with
four thrust-producing rotors). Lower levels (attitude, thrust, motor speed) are
not exposed yet; the architecture allows adding them as new action types.

Validation is strict and controlled: non-finite numbers, unknown agents, wrong
level or limit violations return **HTTP 422** with a reason and never touch the
simulator. Per-agent limits live in the scenario (`limits:`), defaults: 6 m/s
horizontal, 3 m/s vertical, 1.5 rad/s yaw.

Batch: `POST /actions {"drone_01": {"action": ...}, ...}` → `{"ok", "rejected"}`.

## Stepping semantics

* Physics step size is fixed per scenario (`simulation.step_size`, default 4 ms).
* **Actions are held (latched)** until replaced. The environment re-issues each
  agent's current command at every step, so a command survives resets of the
  transport layer and applies for the whole step.
* `POST /simulation/step {"steps": N, "actions": {...}}`
  1. validates and applies the actions (rejections are reported, the step still runs),
  2. advances **exactly N** iterations in deterministic 25-iteration chunks whenever the
     environment has something to re-evaluate (kinematic entities are placed for each
     chunk's end time; waypoint controllers are re-run each chunk so they close the loop),
     otherwise in one go; blocks until Gazebo is paused again,
  5. checks bounds/timeout, collects events since the previous step response,
  6. returns `{episode, status, observations, events, rejected_actions}`.
* Sensors update inside the step at their own rates (IMU 200 Hz, NavSat 10 Hz,
  cameras 15 Hz, contacts 50 Hz); the observation returned is the latest sample.
* A late client in **stepped** mode simply delays the world: nothing advances
  without a step. In **realtime** mode the world keeps running and a late client
  sees newer observations.
* Actions travel to Gazebo asynchronously; the step waits a few ms after sending
  so they are applied on the first iteration. Verified: identical `reset → act →
  step(401)` sequences reproduce positions to 7 decimals. Resting contacts jitter
  at ~1e-7 m between resets (physics-engine floor).

## Modes

`POST /simulation/mode {"mode": "realtime" | "stepped"}` (also `?mode=` on load
and `mode` on reset).

* **realtime** — free-running at `real_time_factor`; pause/resume; waypoint
  controllers and environment entities are driven at 20 Hz.
* **stepped** — paused between steps; `resume` is refused (409).

## Episodes

```jsonc
{"episode_id": "20260919-120218-0bd5dd", "scenario_id": "collision_test", "seed": 7,
 "mode": "stepped", "status": "paused", "sim_time": 5.2, "step_count": 27, "iterations": 1301,
 "max_sim_time": null, "log_path": "runs/.../episode_...jsonl"}
```

Status: `created → running/paused → completed | terminated | failed`.
`POST /episode/reset {"seed", "scenario", "mode"}`:

* same scenario & seed → world rewind (fast; ~3 s if cameras are attached because
  the rendering system re-initialises);
* different seed → the Gazebo server is relaunched with `--seed`;
* different scenario → full load.

Episodes end on `timeout` (`episode.max_sim_time`) or on any event listed in
`episode.terminate_on`; the world pauses and further steps return 409 until reset.

Entities spawned at runtime (`POST /entities`) belong to the *current* episode
only: `reset` removes them before rewinding (save the scenario to make them
permanent). Removing an entity advances the world by 2 iterations in stepped
mode (a Gazebo workaround, see ARCHITECTURE.md). If the simulator process dies,
the episode becomes `failed`, a `simulator_crashed` event is emitted and the
next `reset` relaunches the scenario.

## Events

`GET /events?since=<seq>`; also pushed on `/ws` and attached to step responses.

| event | when | data |
|---|---|---|
| `collision` | agent's body touches anything that is not a landing surface (`ground`, `pad_*`) | `position`, `collision`, `depth`, `other_kind` |
| `landing` / `takeoff` | ground-contact state changes (0.35 s hysteresis) | `position` |
| `out_of_bounds` / `in_bounds` | agent leaves/re-enters `world.bounds` | `position` |
| `timeout` | sim time ≥ `max_sim_time` | |
| `agent_spawned` / `agent_removed` | runtime entity changes | |
| `episode_end` | episode completed/terminated | `status`, `reason` |

No rewards, no task logic: consumers interpret raw events.

## Sensor transport

Low-rate state is JSON over `/ws` (poses at ≤30 Hz, events, status). Frames use
a separate path with one authoritative source (the engine's latest frame):

* `GET /agents/{id}/sensors/camera?format=jpeg|png|raw` (raw = rgb8 bytes)
* `GET /agents/{id}/sensors/depth?format=png16|color|raw` (png16 = uint16 mm, raw = float32 m)
* `WS  /ws/sensors/{id}/{sensor}?format=…&fps=…` — per frame: a JSON header
  message then a binary message. Headers `X-Sensor-*` carry width/height/seq/sim_time.

## Logging, metrics

Each episode writes `runs/<run>/episode_<id>.jsonl`: an `episode` header
(scenario, seed, mode), `step` samples (every step in stepped mode, 10 Hz in
realtime: per-agent state, last action, control mode, events) and `event`
records. `GET /metrics`: real-time factor, physics Hz, step size, entity/agent
counts, connected clients, step latency (EMA), sensor fps, event count.


## World state, recording, snapshots, replay (Phase 3b)

| endpoint | purpose |
|---|---|
| `GET /world/state` | privileged full state: sim time, iteration, every entity's state, each agent's latched command / control mode, episode, recording meta |
| `POST /recordings/start {observations, states, frames}` | turn on per-step **rows** for the current episode (a replay buffer). Actions are *always* logged per episode with their iteration stamp |
| `POST /recordings/stop` | stop rows |
| `GET /recordings`, `GET /recordings/{id}` | list / meta + full action log |
| `GET /recordings/{id}/rows?since=&limit=` | pull rows `(i, iteration, sim_time, actions, states, observations, events)` incrementally — external learners fill their own buffers from this |
| `POST /recordings/{id}/replay {until_iteration?}` | deterministic re-run: reset(scenario, seed) then re-apply the action log |
| `POST /snapshots {name}` | capture: entity states + agent commands + pointer into the action log (a copy of the log is stored with the snapshot) |
| `GET /snapshots`, `GET /snapshots/{id}` | list / inspect |
| `POST /snapshots/{id}/restore` | replay-based restore; returns the **divergence** between restored and captured states (0.0 for stepped-mode recordings) |

Recordings live in `runs/<run>/recordings/<episode_id>/{meta.json, actions.jsonl, rows.jsonl}`;
snapshots in `runs/snapshots/`.

**Why replay-based.** gz-sim 10 accepts an ECM state through `/world/<w>/control/state`
but physics does not adopt it mid-episode (verified: the drone kept flying from its
current pose). The world *is* deterministic given scenario + seed + the per-iteration
action sequence, so restore = reset + re-apply. Cost: about 8× faster than real time
(a 20 s episode restores in ~2.5 s). Realtime-mode recordings replay each action at its
recorded iteration, which is approximate because realtime actions arrive between
iterations; the returned divergence tells you how approximate.

A restore (or replay) starts a **new episode** whose recording begins with the replayed
prefix, so every episode's recording is self-contained and reproduces that episode from
a bare reset. Recordings also log `spawn` / `remove` / `teleport` operations, so runtime
entities are re-created on replay. Note that `reset` removes runtime-spawned entities
(and purges the copies gz-sim's rewind resurrects); scenario entities are never touched.

Client: `sim.world_state()`, `sim.recording_start()/stop()`, `sim.recording_rows()`,
`sim.iter_rows()`, `sim.replay(id)`, `sim.snapshot(name)`, `sim.restore(id)`.
