# Autonomous Simulation World

## Development Brief — Phase 3: First Learned Agent — Visual Navigation

### 1. Objective

Introduce the first actual AI agent into the simulation.

Phase 1 established the physical simulation.

Phase 2 established the external agent interface.

Phase 3 introduces:

```text
                  ┌──────────────────┐
                  │   AI Controller  │
                  │                  │
                  │ Policy / Model   │
                  └────────┬─────────┘
                           │
                        action
                           ↓
                  ┌──────────────────┐
                  │ Simulation API   │
                  └────────┬─────────┘
                           ↓
                  ┌──────────────────┐
                  │    Gazebo Sim    │
                  │                  │
                  │ Drone + World    │
                  └────────┬─────────┘
                           │
                       observation
                           ↓
                  ┌──────────────────┐
                  │   AI Controller  │
                  └──────────────────┘
```

The first experiment should answer a deliberately simple question:

> **Can an agent learn to navigate a physical simulated environment from observations rather than being explicitly programmed with a navigation controller?**

This is the first step toward later experiments involving:

* learned control
* model-based RL
* world models
* planning
* dynamic targets
* multi-agent coordination
* swarm behavior

The focus of this phase is **not** to solve the eventual drone-swarm problem.

The focus is to establish a reliable experimental loop for learning.

---

# 2. Phase 3 Principle

Keep the first learning problem intentionally small.

The environment already supports:

```text
physics
observations
actions
episodes
reset
step control
events
multiple agents
logging
```

Do not redesign those systems around a particular learning algorithm.

Instead:

```text
Simulation
      ↑
      │
 generic interface
      │
      ↓
Learning system
```

The simulator should remain unaware of:

* PPO
* SAC
* DQN
* Dreamer
* world models
* transformers
* CNNs
* policy gradients
* reward algorithms

The learning system should interact with the environment through the Phase 2 interface.

---

# 3. First Task — Point-to-Point Navigation

The initial task should be:

```text
Drone starts at position A

        ↓

      [ DRONE ]

        ↓

   Navigate through
   the environment

        ↓

     [ TARGET ]
```

The drone must reach a designated target location.

A successful episode occurs when:

```text
distance(drone, target) < threshold
```

Possible failure conditions:

```text
collision
out_of_bounds
timeout
```

Do not introduce moving targets yet.

Do not introduce multiple drones as part of the learning problem yet.

---

# 4. Task Randomization

The task should not consist of one fixed:

```text
start → target
```

configuration.

Randomize:

### Starting position

Within a valid region.

### Target position

Within another valid region.

### Environment configuration

Eventually vary:

* obstacle layout
* obstacle positions
* target location
* starting orientation
* environmental conditions

The first version can keep the world geometry fixed while randomizing start and target positions.

The important principle is:

> The agent should learn a task, not memorize one trajectory.

---

# 5. Curriculum of Difficulty

Do not begin with a difficult navigation problem.

Introduce complexity progressively.

### Level 1 — Open space

```text
Drone ─────────────→ Target
```

No meaningful obstacles.

Goal:

> Learn basic movement and target reaching.

---

### Level 2 — Sparse obstacles

Introduce a small number of obstacles.

```text
Drone ────┐
          │
       ███│
          │
          └────→ Target
```

Goal:

> Learn basic obstacle avoidance.

---

### Level 3 — Structured environment

Use the buildings and structures already present in the simulation.

Goal:

> Navigate through a realistic spatial environment.

---

### Level 4 — Randomized layouts

Vary obstacle placement while preserving valid navigation paths.

Goal:

> Learn general navigation rather than memorizing geometry.

---

# 6. First Observation Space

Begin with privileged state observations.

For example:

```text
drone position
drone velocity
drone orientation
target relative position
```

This is intentional.

The first experiment should separate:

> **learning control**

from:

> **learning perception**

If the agent cannot navigate using clean state information, introducing camera perception will make debugging substantially harder.

---

# 7. Second Observation Space — Navigation Sensors

Once state-based navigation works, introduce observations closer to what a real autonomous drone would have.

For example:

```text
GPS
IMU
velocity
target-relative information
```

The exact observation profile should use the existing Phase 2 observation system.

The experiment becomes:

```text
Can the learned controller navigate
using realistic navigation observations?
```

---

# 8. Third Observation Space — Vision

Only after the previous experiments work should the agent receive visual observations.

For example:

```text
RGB camera
+
IMU
+
target information
```

Eventually the target itself could be represented visually rather than supplied explicitly.

This creates a progression:

```text
Ground-truth state
       ↓
Navigation sensors
       ↓
Visual observations
       ↓
Visual autonomous navigation
```

This progression is important because failures can be attributed to the appropriate layer.

---

# 9. Action Space

Use the existing Phase 2 high-level action abstraction.

Initially:

```text
desired velocity
```

or the equivalent control interface already implemented by the simulator.

Do not begin with:

```text
motor commands
thrust
torque
```

The purpose of Phase 3 is to investigate **learning**, not low-level flight dynamics.

The simulator's lower-level controller should continue translating the high-level command into physically plausible movement.

---

# 10. Reward / Objective

Introduce the first task-specific objective outside the simulator.

The simulator should continue to emit raw events and state.

The learning environment may derive a reward such as:

```text
progress toward target
− collision penalty
− time penalty
+ target reached reward
```

The exact reward should be configurable rather than hard-coded into Gazebo.

For example:

```text
Environment
    │
    ├── observation
    ├── state
    ├── events
    │
    ↓
Task wrapper
    │
    ├── reward
    ├── termination
    └── task metrics
    │
    ↓
Learning algorithm
```

This is an important architectural distinction:

> **The simulator describes what happened. The task defines what matters.**

---

# 11. Task Wrapper

Create a lightweight task/environment wrapper around the Phase 2 simulation API.

Conceptually:

```python
env = NavigationTask(sim)

obs = env.reset()

while not done:

    action = policy(obs)

    obs, reward, done, info = env.step(action)
```

The wrapper should translate the generic simulator interface into a learning-friendly interface.

It may provide:

```text
observation
action
reward
terminated
truncated
info
```

But the underlying simulation API remains independent.

---

# 12. First Learning Algorithm

Use a simple, well-understood model-free RL baseline first.

A suitable initial choice is:

```text
PPO
```

The purpose is not to establish PPO as the preferred algorithm.

It is simply a baseline that allows us to establish:

```text
simulation
      ↓
observations
      ↓
policy
      ↓
actions
      ↓
physics
      ↓
reward
      ↓
learning
```

Before introducing world models, we need to prove that the environment can support ordinary learned control.

---

# 13. Baseline Controllers

Maintain the Phase 2 non-learning controllers.

For example:

```text
PID / waypoint controller
```

and compare them against:

```text
random policy
```

and:

```text
learned policy
```

The purpose is diagnostic.

We should be able to distinguish:

```text
Environment problem
Controller problem
Learning problem
```

rather than assuming every failure is a machine-learning failure.

---

# 14. Evaluation

Do not evaluate only on training episodes.

Create separate evaluation scenarios.

Track metrics such as:

### Success rate

Percentage of episodes reaching the target.

### Collision rate

Percentage of episodes resulting in collision.

### Time to target

Simulation time required to reach the target.

### Path efficiency

Compare actual path length against a reasonable reference path.

### Final distance

Distance from target at episode termination.

### Control smoothness

Measure unnecessary oscillation or aggressive control.

### Generalization

Evaluate on start/target configurations not encountered during training.

---

# 15. Deterministic Evaluation Set

Create a fixed evaluation set.

For example:

```text
Evaluation Scenario 01
start = A
target = B

Evaluation Scenario 02
start = C
target = D

Evaluation Scenario 03
start = E
target = F
```

These scenarios should not change between experiments.

This allows meaningful comparisons between:

```text
model v1
model v2
PPO
future world model
future planner
```

---

# 16. Training vs Evaluation Separation

Training should use randomized environments.

Evaluation should use controlled scenarios.

Conceptually:

```text
                 Simulation
                     │
            ┌────────┴────────┐
            │                 │
        Training          Evaluation
            │                 │
      randomized          fixed set
      scenarios           scenarios
            │                 │
            ↓                 ↓
        Learning           Metrics
```

Never judge progress purely from training reward.

---

# 17. Episode Recording

Expand the Phase 2 logging system to support learning experiments.

Record:

```text
episode_id
scenario_id
seed
simulation_time
observation
action
reward
event
termination_reason
```

Where practical, also record:

```text
drone position
velocity
target position
distance to target
```

This should allow an episode to be reconstructed and analyzed later.

---

# 18. Dataset Generation

The simulator should be capable of generating trajectories.

For example:

```text
Episode
    ↓
obs₀
action₀
obs₁
action₁
obs₂
action₂
...
```

This is important beyond RL.

These trajectories will eventually support:

* imitation learning
* supervised dynamics models
* representation learning
* world-model training
* offline RL
* behavioral analysis

The simulator should therefore treat recorded trajectories as a first-class research artifact.

---

# 19. Demonstration Data

Generate a small collection of trajectories using the existing engineered controller.

For example:

```text
waypoint controller
        ↓
successful trajectories
        ↓
dataset
```

These demonstrations can later be used to investigate:

* behavior cloning
* representation learning
* dynamics prediction
* world-model learning

Do not implement those systems yet.

---

# 20. First Learned-Agent Demo

The first compelling demonstration should be visually simple.

Example:

```text
                  TARGET
                    ●
                    ↑
                   /
                  /
                 /
        DRONE ●
```

The browser displays:

* the drone
* target
* trajectory
* obstacles
* current observation
* control action
* episode status

The learned policy controls the drone.

A useful demo should allow:

```text
Reset
   ↓
New start/target
   ↓
Policy controls drone
   ↓
Drone navigates
   ↓
Target reached / failure
```

---

# 21. Observation Comparison

Build a mechanism for switching the same task between observation modes.

For example:

```text
STATE
NAVIGATION
VISION
```

The same task should be executable under each profile.

This allows experiments such as:

> How much does the problem change when we remove privileged state?

The underlying physical environment remains unchanged.

---

# 22. Policy Interface

The policy should be an external component.

Conceptually:

```python
policy = Policy(model)

action = policy(obs)
```

The simulation should not import the model directly.

This makes it possible to later replace:

```text
PPO policy
      ↓
world-model planner
      ↓
MPC controller
      ↓
V-JEPA-based planner
      ↓
LLM-based high-level planner
```

without changing the simulator.

---

# 23. Policy Deployment Modes

Support at least:

### Training mode

The policy interacts with many episodes.

### Evaluation mode

The policy runs without updating weights.

### Demonstration mode

The policy runs visibly through the browser.

The same environment should support all three.

---

# 24. Failure Analysis

Build basic tooling for understanding failed episodes.

When a policy fails, make it possible to determine:

```text
Where did it fail?
Why did it fail?
What did it observe?
What action did it take?
What happened physically?
```

For example:

```text
t = 12.4s
distance = 8.2m
observation = ...
action = ...
velocity = ...
collision = false

t = 12.6s
distance = 8.0m
action = ...
...
```

The goal is not sophisticated explainability.

It is simply to make experimentation inspectable.

---

# 25. Experiment Tracking

Each experiment should record:

```text
experiment_id
algorithm
model configuration
observation profile
action space
reward configuration
scenario configuration
random seeds
training steps
evaluation results
```

This prevents experiments from becoming impossible to reproduce.

---

# 26. First Experimental Ladder

Phase 3 should proceed through increasingly difficult experiments.

### Experiment A

State observations + open environment.

```text
Goal:
learn basic target reaching.
```

### Experiment B

State observations + obstacles.

```text
Goal:
learn obstacle-aware navigation.
```

### Experiment C

Randomized start/target positions.

```text
Goal:
generalize beyond memorized trajectories.
```

### Experiment D

Navigation sensor observations.

```text
Goal:
remove privileged state.
```

### Experiment E

Visual observations.

```text
Goal:
learn navigation from perception.
```

Do not proceed to the next level merely because the previous model trains.

It should meet the evaluation criteria established for that level.

---

# 27. What We Are Learning From This Phase

Phase 3 is not primarily about producing an impressive drone.

It is about establishing several important concepts experimentally.

### 1. The difference between environment and policy

The simulator remains a world.

The policy is external.

### 2. The difference between control and intelligence

A low-level controller can provide stable flight while a learned policy decides where to move.

### 3. The difference between state and observation

The simulator knows more than the agent.

### 4. The difference between task and environment

The simulator exposes events and state.

The task defines reward and success.

### 5. The difference between training and evaluation

A high training reward does not automatically mean generalization.

---

# 28. What We Should NOT Add Yet

Do not implement:

* multi-agent RL
* swarm policies
* MARL
* pursuit
* interception
* adversarial agents
* learned communication
* world models
* Dreamer
* V-JEPA
* Cosmos
* transformer policies
* model predictive control research
* sim-to-real
* PX4
* ArduPilot
* motor-level learning

These become much more interesting once the basic learned-agent loop is reliable.

---

# 29. Future Bridge to World Models

Phase 3 should deliberately prepare for a later world-model experiment.

The trajectory structure should eventually support:

```text
oₜ
aₜ
oₜ₊₁
aₜ₊₁
oₜ₊₂
...
```

This creates the raw material for learning:

```text
representation
     ↓
dynamics
     ↓
future prediction
     ↓
planning
```

The eventual progression could therefore become:

```text
Phase 3

Model-free policy
        ↓
learned navigation


Phase 4

Learned dynamics / world model
        ↓
predict future states


Phase 5

Planning with the world model
        ↓
imagine candidate futures
        ↓
choose actions


Phase 6

Dynamic targets
        ↓
pursuit / interception


Phase 7

Multiple agents
        ↓
coordination


Phase 8

Swarm intelligence
```

This gives the overall project a coherent research arc rather than a collection of unrelated AI demos.

---

# 30. Phase 3 Definition of Done

Phase 3 is complete when:

1. A navigation task can be created on top of the Phase 2 simulator.
2. Start and target positions can be randomized.
3. The task has configurable success/failure conditions.
4. Reward is implemented outside the core simulator.
5. A standard RL baseline can train against the environment.
6. A learned policy can control a simulated drone.
7. The policy operates entirely through the external agent interface.
8. Training and evaluation environments are separated.
9. Evaluation metrics are automatically recorded.
10. Fixed evaluation scenarios can be reproduced.
11. Successful and failed trajectories can be inspected.
12. State-based navigation works reliably.
13. Obstacle navigation can be evaluated.
14. The same task can eventually consume different observation profiles.
15. Trajectories can be recorded as reusable datasets.
16. The browser can visualize a learned policy controlling the drone.
17. The simulator itself contains no RL-specific logic.
18. Re-running an experiment with the same configuration and seed produces reproducible environment conditions.

---

# 31. Final Success Test

The definitive Phase 3 test should look like this:

```text
                  TRAINING

Randomized scenarios
        ↓
Simulation
        ↓
Observations
        ↓
RL policy
        ↓
Actions
        ↓
Physics
        ↓
Reward
        ↓
Policy update
        ↓
repeat
```

After training:

```text
                  EVALUATION

Unseen scenario
        ↓
Simulation
        ↓
Observation
        ↓
Frozen policy
        ↓
Action
        ↓
Drone movement
        ↓
Target reached?
        ↓
Metrics
```

Then the browser should allow someone to watch the resulting policy operate in the physical world.

The critical milestone is not:

> "We trained PPO."

It is:

> **We now have a physical simulation world in which an externally trained agent can perceive, act, learn, and be evaluated.**

At that point, the simulator becomes the foundation for the more interesting question:

> **What changes when the agent is no longer forced to learn a policy directly, but instead learns a model of the world and uses that model to decide what to do?**

That is where the world-model part of the project begins.
