# Learning layer (Phase 3)

`learn/envdr3d_learn/` sits *above* the simulator and talks to it only through
`simclient` (the Phase 2 public API). The simulator has no notion of tasks,
rewards, policies or PPO; it emits state, observations and events.

```
Simulation API  ──observe/act/step/reset──►  NavigationTask  ──obs,reward,done──►  Policy / RL
      │                                          │                                    │
   events, state                        reward, success, metrics,               PPO (SB3), random,
   (what happened)                      trajectory recording                    waypoint baseline
                                        (what matters)
```

## Task: point-to-point navigation

`NavigationTask(sim, TaskConfig(level=..., observation=...))`

| level | world | start region (x) | target region (x) | obstacles |
|---|---|---|---|---|
| `open` | `open_field` | −12…−4 | 6…16 | none |
| `pillars` | `pillars` | −18…−12 | 12…18 | 8 pillars between the regions |
| `city` | `test_city` | −45…−30 | 30…45 | the Phase 1 buildings |

Every `reset()` rewinds the world, then **teleports** the drone to a random start
(random yaw) and the collision-free target marker to a random target, arms the
drone and lets it settle 20 iterations. Fixed evaluation pairs bypass sampling.

**Observation vector (12)** — identical layout under every observation mode:
`rel_target_xyz(3), vel_world_xyz(3), sin/cos(yaw), altitude, distance, target_dir_body_xy(2)`.
What differs is where the numbers come from:

| mode | simulator profile | source of position / velocity / heading |
|---|---|---|
| `state` | `state` | privileged ground truth |
| `navigation` | `navigation` | GPS → local ENU metres, body velocity rotated by IMU heading |
| `vision` | `vision_nav` (camera, depth, imu, gps, velocity) | as navigation; the policy fetches frames itself via `sim.frame()` |

**Action (3)**: `[-1, 1]^3` → world-frame velocity setpoint × `max_speed` (3 m/s).
The simulator's `MulticopterVelocityControl` turns it into rotor thrusts.

**Reward** (`RewardConfig`, outside the simulator): `+progress·Δdistance − time_penalty
− action_penalty·|a|² + reached − collision − out_of_bounds`. Defaults 1.0 / 0.01 /
0.005 / 10 / 5 / 5.

**Termination**: `reached` (distance < 1 m), `collision`, `out_of_bounds`,
`timeout` (episode end), `truncated` (200 task steps = 20 s sim).

## Speed

Nav scenarios run Gazebo with `real_time_factor: 0` (as fast as possible, ≈ 8×
real time for one drone). A task step is 25 physics iterations (0.1 s sim) and
takes ~12 ms wall → **~83 task steps/s per simulator**; reset ≈ 0.45 s.
`SimulatorPool` launches N API+Gazebo pairs on separate ports/partitions for
`SubprocVecEnv`; 4 simulators give ~190 PPO steps/s on an M-series laptop.

## Tools

```bash
export PYTHONPATH=client:learn
python -m envdr3d_learn.evalsets                       # regenerate the fixed eval sets (seeded; do not edit by hand)
python -m envdr3d_learn.evaluate --policy waypoint --evalset level1_open_eval [--observation navigation] [--record]
python -m envdr3d_learn.record   --policy waypoint --level open --episodes 50 --name demos_open_waypoint --only-success
python -m envdr3d_learn.train_ppo --level open --n-envs 4 --steps 150000 --name ppo_open_state
python -m envdr3d_learn.inspect_episode experiments/<eval dir>/episodes.jsonl --failed
python examples/demo_policy.py --policy experiments/<id>/model.zip           # watch it in the browser
```

Artefacts:

* `eval_sets/*.json` — fixed start/target pairs (never change between experiments).
* `experiments/<id>/experiment.json` — algorithm, config, observation profile,
  action space, reward, seeds, git commit; `results` with eval history,
  final evaluation and the baselines measured on the same set.
* `experiments/<id>/evals/*/episodes.jsonl` — per-episode traces
  (t, distance, position, velocity, action, reward, events) for failure analysis.
* `datasets/<name>/episode_XXXXX.npz` — `obs[T+1]`, `action[T]`, `reward[T]`,
  positions, velocities, events, target/start, result; `index.jsonl` summary.
  `obs_t, a_t, obs_{t+1}` alignment is what dynamics/world-model training needs.

## Experimental ladder

| exp | observation | level | status |
|---|---|---|---|
| A | state | open | see README results |
| B | state | pillars | evaluate with `--evalset level2_pillars_eval` |
| C | randomised start/target | (built in: training samples, evaluation uses unseen fixed pairs) | |
| D | navigation | open | baseline validated; train with `--observation navigation` |
| E | vision | open | pipeline validated (frames + task); training needs a CNN policy and a Linux/GPU box for throughput |

Do not advance a level because the model *trains*; advance when its evaluation on
the fixed set meets the level's criterion (open: ≥ 90 % success, 0 collisions).


## Lessons from the first learned agent (failure analysis in practice)

Both problems below were found with `inspect_episode` on evaluation traces, not by
staring at reward curves — which is the point of brief 3 §13/§24.

1. **Stall 8–9 m from the target (run 1).** The policy learned the coarse heading and
   flew fast, but its fine approach was poor. Cause: the progress reward sums to
   `d0 − dT` regardless of speed or final closeness, and the +10 reach bonus was rarely
   found under exploration noise of std ≈ 0.8 (≈ 2.4 m/s). Fix: dense
   `distance_penalty·d` per step, `log_std_init = −0.5`, γ = 0.98. Training return
   then rose monotonically.

2. **Episodes where the drone never moved (run 2, 6/20 evaluation "failures").** The
   drone hovered at its start for 20 s with velocity exactly 0.00 while the policy
   commanded a saturated diagonal `[1, 1, ·]` = 4.24 m/s. The agent's advertised limit
   is 4 m/s, so the simulator rejected the action (correctly, 422) and the task
   ignored `rejected_actions`; the drone kept its latched hold. Environment problem
   (in the task layer), not a learning problem: the engineered baseline never
   saturates and scored 20/20 through the same path. Fix: the task clamps to the
   agent's limits and raises on any rejection. The polluted rollouts also explain the
   noisy, non-monotonic evaluation curve of run 2.

General rule that fell out of this: **when a learned policy fails, first replay the
trace and compare with the engineered baseline on the same fixed pair.** Identical
outcomes point at the environment/task; divergent ones at the policy.
