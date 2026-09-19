# Autonomous Simulation World

## Development Brief — Phase 2: Agent Interface, Control & Environment Interaction

### 1. Objective

Extend the Phase 1 simulation into a robust experimental environment that external programs can interact with as autonomous agents.

Phase 1 established:

```text
Gazebo Sim
    ↓
Physical world
    ↓
Drone
    ↓
Simulation API
    ↓
Browser
```

Phase 2 should establish the complete interaction loop:

```text
                 OBSERVATION
                      ↑
                      │
              ┌───────┴────────┐
              │                │
        External Agent     Browser
              │                │
              ↓                ↓
             ACTION       Human controls
              │
              └───────┬────────┘
                      ↓
                 Simulation
                      ↓
                  New State
```

The objective is to make the simulation suitable for future:

* reinforcement learning
* model-based RL
* world-model experiments
* planning
* autonomous navigation
* multi-agent systems
* swarm experiments
* perception experiments

without implementing those systems yet.

---

# 2. Phase 2 Principle

The simulator should behave like an **environment**, not like an AI application.

The environment owns:

* physics
* state
* observations
* sensors
* collisions
* entities
* simulation time
* world state

The external agent owns:

* decision making
* planning
* learning
* policy
* intelligence

The environment should never need to know whether the external client is:

```text
random controller
PID controller
MPC
PPO
SAC
MAPPO
Dreamer
world model
neural network
human
```

---

# 3. Establish a Formal Agent Interface

Define a consistent interface for every controllable agent.

Conceptually:

```python
observation = agent.observe()

action = external_controller(observation)

agent.act(action)

simulation.step()
```

The implementation may use HTTP/WebSocket or another appropriate protocol, but the conceptual contract must remain stable.

Every agent should have:

```text
agent_id
agent_type
observation_space
action_space
state
sensors
```

---

# 4. Observation System

Create a formal observation system.

The environment should distinguish between:

### Ground-truth state

Information available internally to the simulator:

```text
position
velocity
orientation
angular velocity
acceleration
```

and:

### Agent observation

Information explicitly exposed to an external agent.

For example:

```text
camera image
depth image
IMU
GPS
local velocity
```

This distinction is extremely important.

An external agent must **not automatically receive the complete simulator state**.

The observation interface should be configurable.

---

# 5. Observation Profiles

Introduce configurable observation profiles.

For example:

### State profile

```text
position
velocity
orientation
```

### Navigation profile

```text
GPS
IMU
velocity
```

### Vision profile

```text
RGB camera
depth camera
IMU
```

### Minimal profile

```text
camera
IMU
```

Eventually this will allow experiments such as:

> Can an agent solve the same task using privileged state versus visual observations?

The environment should support changing the observation profile through scenario configuration rather than code changes.

---

# 6. Action System

Formalize the action space.

Initially support a high-level control abstraction such as:

```text
desired velocity
```

and/or:

```text
desired position / waypoint
```

The action representation must be clearly documented.

For example:

```text
Action

vx
vy
vz
yaw_rate
```

or an equivalent representation selected according to the actual Gazebo implementation.

The simulator should validate actions:

* correct dimensions
* valid numerical values
* physical limits
* invalid actions

Invalid actions should produce controlled errors rather than destabilizing the simulation server.

---

# 7. Action Abstraction Layers

Design the system so multiple control levels can eventually exist.

```text
High level
    ↓
Waypoint
    ↓
Velocity
    ↓
Acceleration
    ↓
Attitude
    ↓
Thrust / torque
    ↓
Motor
```

Phase 2 should implement only the first one or two useful levels.

Do not implement motor-level control yet.

The important requirement is that the architecture does not make future lower-level control impossible.

---

# 8. Simulation Step Semantics

Formalize what happens when an external agent submits an action.

The intended loop should be deterministic and understandable:

```text
1. External agent receives observation
2. External agent calculates action
3. Action is submitted
4. Simulation advances
5. Physics updates
6. Sensors update
7. New observation becomes available
```

Document:

* simulation timestep
* action duration
* observation timing
* sensor timing
* whether actions are held between steps
* synchronization behavior
* what happens if an external client is late

This will become extremely important for RL experiments.

---

# 9. Synchronous and Real-Time Modes

Support at least two simulation modes.

### Real-time mode

The simulation attempts to run at real-world speed.

Useful for:

* human interaction
* visualization
* demonstrations

### Step-controlled mode

External software explicitly advances simulation time.

Useful for:

* RL
* planning
* reproducible experiments
* dataset generation

For example:

```text
reset()
→ observation

step(action)
→ observation

step(action)
→ observation

step(action)
→ observation
```

The simulation should not advance unexpectedly while an external experiment is operating in step-controlled mode.

---

# 10. Episode Concept

Introduce the concept of an **episode** even though we are not implementing RL yet.

An episode represents one run of a scenario.

It should have:

```text
episode_id
scenario_id
random_seed
start_time
simulation_time
status
```

Possible status:

```text
created
running
paused
completed
terminated
failed
```

This prepares the environment for future RL without coupling it to an RL framework.

---

# 11. Reset Semantics

A reset should return the simulation to a known state.

Support:

```text
reset()
reset(seed)
reset(scenario)
```

A reset should:

* restore entity positions
* restore velocities
* reset simulation time
* reset sensors
* reset environment variables
* reset random state
* clear transient telemetry

The result should be reproducible when the same seed and scenario are used.

---

# 12. Termination and Events

Introduce an environment event system.

The simulator should be capable of detecting events such as:

```text
collision
out_of_bounds
agent_destroyed
agent_removed
target_reached
timeout
landing
takeoff
```

Do not build complex task-specific reward functions.

Instead expose **raw environment events**.

For example:

```json
{
  "event": "collision",
  "entities": ["drone_01", "building_03"],
  "simulation_time": 42.31
}
```

Future task implementations can interpret these events however they want.

---

# 13. Collision System

Make collisions observable and useful.

The system should expose:

* collision occurrence
* entities involved
* location
* simulation time
* relevant physical information where available

The browser should optionally visualize collisions/debug information.

This will eventually be useful for:

* obstacle avoidance
* drone coordination
* swarm safety
* adversarial scenarios

---

# 14. Multiple Agents

Phase 2 should properly support multiple drones.

For example:

```text
drone_01
drone_02
drone_03
drone_04
```

Each drone should independently have:

* state
* observations
* sensors
* action interface
* telemetry

The external API should make it possible for one external controller to operate:

```text
one drone
```

or:

```text
many drones
```

without changing the fundamental API.

---

# 15. Multi-Agent Observation

The architecture should support different information models.

For example:

### Independent observation

Drone 01 sees only its own sensors.

```text
Drone 01
   ↓
own observation
```

### Shared state

Each agent receives information about other agents.

```text
Drone 01
   ↓
own state + nearby drones
```

### Centralized observation

An external controller can request the state of the entire swarm.

This is useful for future centralized training / decentralized execution experiments.

Do not implement sophisticated MARL yet.

Simply make these observation configurations possible.

---

# 16. Environment Interaction

Introduce basic dynamic objects.

The world should no longer consist exclusively of static buildings.

Add a small number of simple dynamic entities such as:

* moving vehicle
* moving object
* simple target

The purpose is to verify that the environment supports a world that changes independently of the drone's actions.

For example:

```text
Drone
  ↓
observes
  ↓
moving vehicle
  ↓
vehicle continues moving
```

This establishes the foundation for future pursuit/interception tasks.

---

# 17. Basic Environment Controller

Introduce an environment-level controller capable of controlling non-agent entities.

For example:

```text
Target
    ↓
trajectory controller
    ↓
position(t)
```

The first target can simply follow:

* waypoint sequence
* circular trajectory
* straight-line trajectory
* randomized trajectory

This is intentionally not an AI system.

It is a deterministic/environment-controlled dynamic entity.

---

# 18. Browser Improvements

Extend the Phase 1 frontend.

### Multi-agent visualization

Show multiple drones simultaneously.

### Agent selection

Click an agent and inspect:

```text
position
velocity
orientation
sensor status
control mode
```

### Follow camera

Allow following a selected agent.

### Camera modes

Provide:

```text
Free camera
Top-down
Follow agent
First-person camera
```

### Trajectory visualization

Display recent movement trails.

Example:

```text
          ·
        ·
      ·
    ●
```

### Velocity vectors

Optionally visualize:

```text
position ───────→ velocity
```

### Debug overlays

Allow toggling:

* collision geometry
* sensor frustums
* coordinate axes
* trajectories
* velocity vectors
* entity IDs

---

# 19. Sensor Streaming

The browser should be able to display an agent's sensor output.

At minimum:

### RGB

Display the drone camera.

### Depth

Display depth visualization.

### IMU/state

Display numerical telemetry.

The sensor pipeline should be designed so the external AI client can receive the same underlying observations.

Avoid creating a separate browser-only sensor implementation.

There should be one authoritative sensor source.

---

# 20. Observation Transport

The system needs to distinguish between:

### Low-frequency state data

Suitable for:

* JSON
* WebSocket messages

### High-bandwidth sensor data

Such as:

* RGB frames
* depth frames

These may require a more efficient transport/encoding mechanism.

The implementation agent should investigate the appropriate approach rather than simply sending raw uncompressed images through the same channel as telemetry.

The goal is to avoid creating an architecture that becomes unusable once multiple cameras/drones are active.

---

# 21. External Python Client

Create a small official Python client library.

Conceptually:

```python
from simulation_client import Simulation

sim = Simulation("localhost")

episode = sim.reset(seed=42)

obs = sim.observe("drone_01")

while not episode.done:

    action = controller(obs)

    obs = sim.step(
        agent="drone_01",
        action=action
    )
```

The actual API can differ, but it should be:

* simple
* typed where practical
* documented
* independent of Gazebo APIs

The client should hide WebSocket/HTTP implementation details.

---

# 22. Example Controllers

Create several extremely simple controllers for testing.

### Controller 1 — Hover

Maintain position.

### Controller 2 — Waypoint

Move to a specified waypoint.

### Controller 3 — Circular trajectory

Follow a predefined trajectory.

### Controller 4 — Multi-drone formation

Have several drones follow predefined relative positions.

These are **engineering test controllers**, not learned policies.

They exist to prove that the external-agent interface works.

---

# 23. Browser vs External Control

The browser should be capable of acting as a simple human control client for debugging.

For example:

```text
Keyboard
   ↓
Browser
   ↓
Simulation API
   ↓
Drone
```

But browser control should use the same underlying action API as an external Python controller.

There should not be a special hidden control path.

This provides a useful test:

> If the browser can control a drone through the public API, an external AI controller should be able to do the same.

---

# 24. Logging

Every simulation run should be able to produce structured logs containing:

```text
timestamp
simulation_time
agent_id
state
action
events
scenario
seed
```

Sensor recordings can be optional because of their size.

The logging architecture should eventually support ML dataset generation.

Do not build a sophisticated dataset management system yet.

---

# 25. Performance

The environment should expose basic performance metrics:

```text
simulation FPS
real-time factor
simulation timestep
API latency
number of entities
sensor FPS
```

The browser should display basic simulation health.

This will become important when scaling from:

```text
1 drone
```

to:

```text
10 drones
```

and eventually much larger environments.

---

# 26. Testing

Create automated tests for the environment API.

At minimum test:

### Lifecycle

```text
create
reset
pause
resume
step
shutdown
```

### Agent

```text
spawn
observe
act
state
remove
```

### Reproducibility

Same:

```text
scenario + seed
```

should produce equivalent initial conditions.

### Multiple agents

Verify independent observations/actions.

### Collision events

Verify collisions are correctly surfaced.

### External client

Verify a Python client can:

```text
connect
reset
observe
act
step
disconnect
```

---

# 27. Phase 2 Definition of Done

Phase 2 is complete when the following works end-to-end:

```text
                 Browser
                    │
                    │
             Simulation API
                    │
             ┌──────▼──────┐
             │   Gazebo    │
             │             │
             │ Drone 01    │
             │ Drone 02    │
             │ Target      │
             └─────────────┘
                    ▲
                    │
             Python Client
                    ▲
                    │
              Controller
```

Specifically:

1. An external Python process can connect to the simulator.
2. It can reset a scenario.
3. It can receive an observation.
4. It can submit an action.
5. The simulation advances one controlled step.
6. It receives the resulting observation.
7. Multiple drones can be controlled independently.
8. RGB/depth/state observations are available through the agent interface.
9. The browser visualizes those same agents and sensor streams.
10. Simulation can operate in real-time or step-controlled mode.
11. Episodes can be created/reset/reproduced.
12. Environment events such as collisions are exposed.
13. Dynamic entities can exist independently of the controlled drones.
14. Basic external controllers can successfully navigate the drones.
15. The entire system works without any RL/AI framework.
16. Gazebo remains completely independent of the browser and external AI clients.

---

# 28. Explicit Non-Goals

Do not implement:

* PPO
* SAC
* MAPPO
* RL training
* learned policies
* swarm optimization
* world models
* neural networks
* autonomous target pursuit
* intelligent adversaries
* communication-learning
* reward engineering
* sim-to-real
* PX4
* ArduPilot

Those belong to subsequent experiments.

The objective of Phase 2 is to establish a **clean, deterministic, externally controllable simulation environment** capable of supporting all of them later.

---

# 29. Success Test

At the end of Phase 2, a developer should be able to run:

```text
Simulation server
        +
Browser
        +
Python controller
```

and see:

```text
1. Browser opens simulation.
2. World loads.
3. Three drones spawn.
4. Python controller connects.
5. Controller receives observations.
6. Controller sends actions.
7. Drones move through the physical world.
8. Browser displays their movement in real time.
9. One drone collides with an obstacle.
10. Collision event is reported.
11. Simulation can be paused.
12. Simulation can be stepped manually.
13. Simulation can be reset with the same seed.
14. The same initial state is reproduced.
```

If this works reliably, the environment is ready for the next layer:

> **using actual AI to control agents inside the world.**
