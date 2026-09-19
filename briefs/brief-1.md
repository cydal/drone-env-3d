# Autonomous Simulation World

## Development Brief — Phase 1: Physical Simulation Platform

### 1. Objective

Build a standalone, extensible 3D simulation environment that will serve as a long-term experimental playground for AI, reinforcement learning, world models, multi-agent systems, and autonomous-agent research.

The environment should initially focus on autonomous aerial vehicles, particularly drones, but the architecture must not hard-code the project around drones or any particular AI model.

The key principle is:

> **The simulation environment is independent of the intelligence controlling it.**

AI models, RL algorithms, controllers, planners, and world models will run externally and communicate with the simulation through a clean programmatic interface.

The first development phase is focused on establishing the **physical simulation layer and browser-based visualization**. Do not build the AI/control layer yet.

---

# 2. Core Technology Choice

Use:

**Gazebo Sim** as the simulation and physics engine.

The system should run Gazebo in a standalone/headless-capable configuration and expose simulation state and controls through an API/WebSocket layer.

A custom browser frontend should visualize the simulation.

### Do NOT introduce initially

* ArduPilot
* PX4
* RLDroneSim
* Stable-Baselines
* Ray/RLlib
* PettingZoo
* PPO/SAC/MAPPO
* Dreamer
* V-JEPA
* any specific AI/model framework

These may become useful integrations later, but the first phase should establish a clean simulation platform independent of them.

---

# 3. High-Level Architecture

Design the system approximately as:

```text
                         WEB BROWSER
                              │
                              │ WebSocket / HTTP
                              │
                    ┌─────────▼─────────┐
                    │  Simulation API   │
                    │                   │
                    │ state             │
                    │ commands          │
                    │ scenarios         │
                    │ telemetry         │
                    │ simulation time   │
                    └─────────┬─────────┘
                              │
                              │
                       ┌──────▼──────┐
                       │  Gazebo Sim │
                       │             │
                       │ Physics     │
                       │ World       │
                       │ Sensors     │
                       │ Entities    │
                       └──────┬──────┘
                              │
                 ┌────────────┼────────────┐
                 │            │            │
              World         Agents       Sensors
                 │            │            │
             terrain       drones        camera
             buildings     vehicles      depth
             obstacles     targets       LiDAR
             environment                  IMU
```

Later, external AI systems should be able to connect to exactly the same Simulation API:

```text
                     SIMULATION API
                           │
          ┌────────────────┼────────────────┐
          │                │                │
       Browser          RL Agent       World Model
          │                │                │
      visualize         control          plan
```

The browser must therefore **not become the source of truth** for simulation state. Gazebo remains authoritative.

---

# 4. Design Principles

### 4.1 Simulation first

The environment must be usable without any AI model.

A human should be able to:

* launch a world
* view it in the browser
* pause/resume simulation
* reset it
* inspect entities
* inspect telemetry
* manipulate simulation state where appropriate

### 4.2 External intelligence

No learning algorithm should be embedded into the simulation.

The simulator provides:

**Observation → Action → Next state**

External systems provide the intelligence.

### 4.3 Extensibility

The architecture should allow new:

* vehicles
* robots
* targets
* buildings
* environments
* sensors
* physics parameters
* scenarios

without restructuring the entire system.

### 4.4 Determinism where possible

Support reproducible simulation runs through:

* explicit random seeds
* controlled simulation timestep
* deterministic scenario initialization where possible
* recorded configuration

This will become important for RL experiments.

### 4.5 Headless operation

The simulation should be capable of running without a desktop GUI.

The browser is the visualization layer.

This is important because later AI experiments may run remotely or in batch.

---

# 5. Initial World

Create a visually appealing but technically simple initial world.

Do NOT spend excessive effort creating a photorealistic environment at this stage.

The first world should contain:

* large navigable outdoor area
* ground/terrain
* several buildings
* walls/obstacles
* open areas
* elevated structures
* different elevations
* landing/takeoff areas
* sufficient empty space for aerial navigation

The environment should feel like the beginning of a real world rather than a tiny robotics test box.

A coherent visual identity is desirable.

Think:

> **small futuristic autonomous-operations test city**

rather than:

> generic Gazebo demo world.

However, functionality is more important than visual polish during this phase.

---

# 6. Drone Entity

Create a basic drone entity suitable for future autonomous-control experiments.

The drone should have a physically meaningful model rather than simply being a floating visual object.

At minimum expose:

### State

* position
* orientation
* linear velocity
* angular velocity
* acceleration where available

### Physical properties

* mass
* inertia
* collision geometry
* gravity
* aerodynamic parameters where supported

### Control abstraction

For Phase 1, expose a relatively high-level control interface.

Do NOT require an external AI model to directly manipulate individual motor RPMs.

The initial interface may use something such as:

```text
desired velocity
desired acceleration
desired position / waypoint
```

with an appropriate lower-level controller handling the actual vehicle dynamics.

The exact abstraction should be selected based on Gazebo's available drone models/controllers and should remain replaceable.

The architecture must eventually allow lower-level control to be introduced.

---

# 7. Sensors

The drone should have configurable simulated sensors.

Initially implement:

### RGB camera

Configurable:

* resolution
* field of view
* frame rate
* orientation

### Depth camera

For future perception experiments.

### IMU

Expose:

* acceleration
* angular velocity
* orientation information where appropriate

### Position/GPS-like sensor

For initial experimentation.

### Optional future sensors

Design the sensor system so these can later be added without changing the external API:

* LiDAR
* segmentation camera
* optical flow
* radar
* proximity sensors
* event cameras

The important design principle is that the external agent should receive **observations**, not direct access to Gazebo internals.

---

# 8. Simulation API

Design a clean API between the simulation and external clients.

The API should support at minimum:

### Lifecycle

```text
create_simulation()
reset()
pause()
resume()
step()
shutdown()
```

### World

```text
load_world()
load_scenario()
get_world_state()
```

### Entities

```text
list_entities()
get_entity_state(entity_id)
spawn_entity(...)
remove_entity(entity_id)
```

### Agents

```text
get_observation(agent_id)
send_action(agent_id, action)
```

### Simulation

```text
get_simulation_time()
get_real_time_factor()
get_step_count()
```

### Telemetry

```text
subscribe_to_state(...)
subscribe_to_sensor(...)
```

The exact API technology can be chosen by the implementation agent.

A combination of:

* REST/HTTP for configuration and lifecycle
* WebSocket for streaming state/sensor data

is a reasonable starting point.

---

# 9. Browser Frontend

Build a dedicated web application rather than relying entirely on Gazebo's default interface.

The frontend should eventually become the primary way a human interacts with the simulation.

The initial UI should provide:

### Main 3D viewport

Display:

* world
* terrain
* buildings
* drones
* cameras
* basic lighting/environment

### Camera controls

Support:

* orbit
* pan
* zoom
* free camera
* follow selected drone
* top-down view

### Simulation controls

```text
Play
Pause
Reset
Step
Simulation speed
```

### Entity panel

Show:

```text
Entities
  ├── Drone 01
  ├── Drone 02
  ├── Building 01
  └── ...
```

Selecting a drone should display basic telemetry.

For example:

```text
Drone 01

Position
X: ...
Y: ...
Z: ...

Velocity
X: ...
Y: ...
Z: ...

Orientation
...

Status
...
```

### Sensor view

Selecting a drone should allow the user to inspect its camera/depth/sensor observations.

---

# 10. Visual Quality

The application should have a strong visual identity.

It should feel like a serious autonomous-systems simulation environment, not a raw robotics debugging tool.

Priorities:

1. Clear 3D visualization
2. Good lighting
3. Good camera controls
4. Clean UI
5. Useful telemetry
6. Visual distinction between entities
7. Debug overlays

Do not sacrifice simulation architecture for visual effects.

The visual layer must remain replaceable/independent from the physics engine.

---

# 11. Scenario System

Do not hard-code one simulation configuration.

Introduce the concept of a **Scenario**.

A scenario should describe things such as:

```yaml
world:
  name: coastal_city

agents:
  - type: drone
    id: drone_01
    spawn: ...

environment:
  wind: ...
  visibility: ...
  time_of_day: ...

simulation:
  timestep: ...
  seed: ...
```

The exact format can be JSON/YAML or another suitable configuration system.

The important requirement is that a scenario can be saved, loaded, reproduced and modified.

Eventually we should be able to create scenarios such as:

```text
Navigation
Formation
Search
Pursuit
Interception
Encirclement
Multi-agent coordination
```

Do not implement all of these now.

Build the architecture that makes them possible.

---

# 12. Recording and Replay

This should be considered part of the foundational architecture.

The environment will eventually be used to generate datasets for machine learning.

Therefore design for recording:

* simulation state
* actions
* observations
* timestamps
* random seed
* scenario configuration

A run should eventually be replayable.

For example:

```text
Run 001
Scenario: test_city_navigation
Seed: 18372

Drone 01
  observation_t0
  action_t0
  state_t1
  observation_t1
  action_t1
  ...
```

Do not build a sophisticated dataset-generation system in Phase 1, but do not make future recording impossible.

---

# 13. External Client Example

The eventual user experience should make it possible for an external Python process to do something conceptually like:

```python
sim = SimulationClient("localhost")

sim.reset()

while not sim.done():

    observation = sim.get_observation("drone_01")

    action = external_model(observation)

    sim.send_action("drone_01", action)

    sim.step()
```

The external model should have no knowledge of Gazebo APIs.

It should communicate only through our simulation interface.

This abstraction is critical.

---

# 14. Multi-Agent Readiness

Phase 1 does not need sophisticated multi-agent behavior.

However, the architecture must support multiple independent agents from the beginning.

Avoid designs such as:

```text
the_drone
```

Instead use:

```text
agents:
    drone_01
    drone_02
    drone_03
```

Each should have:

* unique ID
* state
* sensors
* observation stream
* action interface

This will allow future multi-agent experiments without rebuilding the environment.

---

# 15. Target/Non-Drone Entities

Do not assume everything in the world is an autonomous drone.

The entity architecture should eventually support:

```text
Drone
Vehicle
Ship
Robot
Target
Obstacle
Building
Static object
Dynamic object
```

For Phase 1, it is sufficient to implement:

* drone
* static environment objects

But design the entity system generically.

---

# 16. Development Phases

### Phase 1A — Simulation foundation

Deliver:

* Gazebo installation/configuration
* headless-capable simulation
* initial world
* basic drone
* physics
* basic sensors
* reproducible reset
* simulation lifecycle

### Phase 1B — Simulation API

Deliver:

* external client connection
* state retrieval
* observation retrieval
* action submission
* stepping
* reset/pause/resume
* entity discovery
* telemetry streaming

### Phase 1C — Browser visualization

Deliver:

* 3D world
* drone visualization
* camera controls
* simulation controls
* entity selection
* telemetry
* sensor visualization

### Phase 1D — Integration test

Create a very simple external controller:

```text
external Python process
        ↓
Simulation API
        ↓
Drone
        ↓
Gazebo physics
        ↓
Browser visualization
```

The controller should make the drone perform a basic deterministic task such as:

> Take off → move to waypoint → hover → land.

This is an integration test, not an AI experiment.

---

# 17. Definition of Done

Phase 1 is complete when:

1. Gazebo runs the simulation independently of the browser.
2. The browser can connect to the simulation.
3. The browser renders the world in 3D.
4. At least one physically simulated drone exists.
5. The drone has configurable simulated sensors.
6. Simulation can be started, paused, reset and stepped.
7. Simulation state can be queried externally.
8. External clients can send actions to a drone.
9. The browser reflects externally commanded movement in real time.
10. Multiple drones can be instantiated.
11. A scenario can be saved and reproduced.
12. A simple external Python controller can successfully control a drone.
13. No AI/RL framework is required by the simulator.
14. The simulator can run headlessly.
15. The codebase is structured so the browser, simulation engine and external AI clients are independently replaceable.

---

# 18. Important Non-Goals

Do NOT attempt to build these in the first phase:

* reinforcement learning
* swarm intelligence
* autonomous pursuit
* world models
* V-JEPA
* Dreamer
* neural networks
* sophisticated flight controllers
* realistic military/weapon systems
* photorealistic graphics
* full autonomous navigation
* realistic adversarial agents
* sim-to-real deployment

Those are future experiments.

The purpose of this phase is to build the **world in which those experiments can happen**.

---

# 19. Final Architectural Principle

The project should be thought of as:

> **A general-purpose physical simulation environment with a browser interface and an external-agent API.**

Not:

> "a drone RL project."

The drone is simply our first interesting entity.

The long-term goal is an extensible simulated world where different AI systems can enter the same environment and be evaluated under the same physical conditions.

Build the foundation accordingly.
