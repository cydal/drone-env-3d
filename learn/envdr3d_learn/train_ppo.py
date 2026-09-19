"""Train a PPO baseline against the navigation task using N parallel simulators.

    python -m envdr3d_learn.train_ppo --level open --n-envs 4 --steps 150000 --name ppo_open_state

Everything that matters for reproducibility lands in experiments/<id>/: experiment.json
(config, seeds, git commit), checkpoints, tensorboard logs, periodic evaluations on the fixed
evaluation set, and the final model.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from .evaluate import run_eval
from .evalsets import load as load_evalset
from .experiment import Experiment
from .gym_env import NavigationGymEnv, SimulatorPool
from .policies import SB3Policy, WaypointPolicy, RandomPolicy
from .task import TaskConfig

EVALSET_FOR_LEVEL = {"open": "level1_open_eval", "pillars": "level2_pillars_eval", "city": "level3_city_eval"}
OBS_SCALE = np.array([20, 20, 10, 5, 5, 5, 1, 1, 10, 30, 1, 1], dtype=np.float32)   # 12-d base layout


def obs_scale_for(cfg: TaskConfig) -> np.ndarray:
    return np.array(OBS_SCALE.tolist() + [20, 20, 5, 10] * cfg.obstacle_features, dtype=np.float32)


def make_env_fn(cfg: TaskConfig, port: int, seed: int):
    def _f():
        import gymnasium as gym
        from stable_baselines3.common.monitor import Monitor
        scale = obs_scale_for(cfg)
        env = NavigationGymEnv(cfg, port=port)
        env = gym.wrappers.TransformObservation(env, lambda o: (o / scale).astype(np.float32),
                                                gym.spaces.Box(-np.inf, np.inf, (len(scale),), np.float32))
        env = Monitor(env)               # logs ep_rew_mean / ep_len_mean to tensorboard + stdout
        env.reset(seed=seed)
        return env
    return _f


class ScaledPolicy:
    """Adapter so the evaluator (raw observations) can drive the SB3 model (scaled observations)."""
    def __init__(self, inner: SB3Policy, name: str, scale=None) -> None:
        self.inner, self.name, self.scale = inner, name, (OBS_SCALE if scale is None else scale)
    def __call__(self, obs): return self.inner(obs / self.scale)
    def reset(self): pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", default="open"); ap.add_argument("--observation", default="state")
    ap.add_argument("--n-envs", type=int, default=4); ap.add_argument("--steps", type=int, default=150_000)
    ap.add_argument("--seed", type=int, default=0); ap.add_argument("--name", default=None)
    ap.add_argument("--eval-every", type=int, default=25_000); ap.add_argument("--eval-episodes", type=int, default=10)
    ap.add_argument("--max-steps", type=int, default=200); ap.add_argument("--base-port", type=int, default=8101)
    ap.add_argument("--lr", type=float, default=3e-4); ap.add_argument("--n-steps", type=int, default=512)
    ap.add_argument("--batch-size", type=int, default=256); ap.add_argument("--ent-coef", type=float, default=0.0)
    ap.add_argument("--gamma", type=float, default=0.98); ap.add_argument("--log-std-init", type=float, default=-0.5,
                    help="initial action noise: std = exp(x) in normalised action units (default 0.6 -> 1.8 m/s)")
    ap.add_argument("--resume", default=None, help="continue training from this model.zip (new experiment dir)")
    ap.add_argument("--obstacle-features", type=int, default=None,
                    help="K nearest obstacles in the observation (default: 0 for open, 3 otherwise)")
    ap.add_argument("--collision-penalty", type=float, default=None, help="default 5 (open) / 25 (obstacle levels)")
    ap.add_argument("--proximity-penalty", type=float, default=None, help="default 0 (open) / 0.1 per m inside the safety margin")
    ap.add_argument("--safe-margin", type=float, default=1.5, help="m from the obstacle surface where the proximity penalty starts")
    ap.add_argument("--lr-decay", action="store_true", help="linear learning-rate decay to 0 over the run")
    a = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv
    import torch
    torch.set_num_threads(2)

    k = a.obstacle_features if a.obstacle_features is not None else (0 if a.level == "open" else 3)
    from .task import RewardConfig
    obstacle_level = a.level != "open"
    reward = RewardConfig(collision=a.collision_penalty if a.collision_penalty is not None else (15.0 if obstacle_level else 5.0),
                          proximity_penalty=a.proximity_penalty if a.proximity_penalty is not None else (0.1 if obstacle_level else 0.0),
                          safe_margin=a.safe_margin)
    cfg = TaskConfig(level=a.level, observation=a.observation, max_steps=a.max_steps, seed=1, obstacle_features=k, reward=reward)
    scale = obs_scale_for(cfg)
    name = a.name or f"ppo_{a.level}_{a.observation}"
    exp = Experiment(name, {
        "algorithm": "PPO (stable-baselines3)", "task": cfg.to_json(), "observation_profile": a.observation,
        "action_space": "Box(-1,1)^3 -> world-frame velocity * max_speed", "obs_scale": scale.tolist(),
        "reward": cfg.reward.__dict__, "n_envs": a.n_envs, "total_steps": a.steps, "seed": a.seed,
        "ppo": {"learning_rate": a.lr, "n_steps": a.n_steps, "batch_size": a.batch_size, "ent_coef": a.ent_coef,
                "gamma": a.gamma, "gae_lambda": 0.95, "policy": "MlpPolicy [64,64]", "log_std_init": a.log_std_init},
        "evalset": EVALSET_FOR_LEVEL[a.level], "eval_every": a.eval_every, "lr_decay": a.lr_decay,
    })
    print("experiment:", exp.dir)

    pool = SimulatorPool(a.n_envs, base_port=a.base_port)
    eval_sim_port = pool.ports[0]
    try:
        venv = SubprocVecEnv([make_env_fn(cfg, p, a.seed + i) for i, p in enumerate(pool.ports)], start_method="spawn")
        lr = (lambda progress: a.lr * progress) if a.lr_decay else a.lr
        if a.resume:
            model = PPO.load(a.resume, env=venv, device="cpu", learning_rate=lr, tensorboard_log=str(exp.path("tb")))
            exp.meta["config"]["resumed_from"] = str(a.resume); exp.save()
        else:
            model = PPO("MlpPolicy", venv, learning_rate=lr, n_steps=a.n_steps, batch_size=a.batch_size, ent_coef=a.ent_coef,
                        gamma=a.gamma, gae_lambda=0.95, seed=a.seed, verbose=1, tensorboard_log=str(exp.path("tb")),
                        policy_kwargs={"net_arch": [64, 64], "log_std_init": a.log_std_init}, device="cpu")
        evalset = load_evalset(EVALSET_FOR_LEVEL[a.level]); evalset["name"] = EVALSET_FOR_LEVEL[a.level]
        evalset_small = {**evalset, "pairs": evalset["pairs"][:a.eval_episodes]}

        class PeriodicEval(BaseCallback):
            def __init__(self):
                super().__init__(); self.last = 0; self.history = []
            def _on_step(self) -> bool:
                if self.num_timesteps - self.last >= a.eval_every:
                    self.last = self.num_timesteps
                    self._eval()
                return True
            def _eval(self):
                # pause training envs' use of sim 0 is not possible while SubprocVecEnv holds it; use a
                # dedicated extra simulator for evaluation instead.
                from simclient import Simulation
                pol = ScaledPolicy(SB3Policy(self.model, deterministic=True), name=f"{name}@{self.num_timesteps}", scale=scale)
                s = run_eval(pol, cfg, evalset_small, sim=Simulation(port=eval_pool.ports[0], timeout=120),
                             out_dir=exp.path("evals", f"step_{self.num_timesteps:08d}"), verbose=False)
                self.history.append({"timesteps": self.num_timesteps, **{k: s[k] for k in ("success_rate", "collision_rate", "mean_final_distance_m", "mean_return", "mean_time_to_target_s")}})
                exp.log_result("eval_history", self.history)
                print(f"[eval @ {self.num_timesteps}] success {s['success_rate']:.2f} collision {s['collision_rate']:.2f} "
                      f"final {s['mean_final_distance_m']:.2f} m return {s['mean_return']:.1f}")
                self.model.save(str(exp.path("checkpoints", f"model_{self.num_timesteps:08d}.zip")))

        eval_pool = SimulatorPool(1, base_port=a.base_port + 50, partition_prefix="envdr3d-eval")
        try:
            t0 = time.time()
            model.learn(total_timesteps=a.steps, callback=[PeriodicEval()], progress_bar=False)
            train_time = time.time() - t0
            model.save(str(exp.path("model.zip")))
            exp.log_result("train_wall_time_s", round(train_time, 1))
            exp.log_result("steps_per_second", round(a.steps / train_time, 1))

            # final evaluation on the full fixed set, alongside the baselines for context
            from simclient import Simulation
            sim = Simulation(port=eval_pool.ports[0], timeout=120)
            final = run_eval(ScaledPolicy(SB3Policy(model), name=name, scale=scale), cfg, evalset, sim=sim,
                             out_dir=exp.path("evals", "final"), record=True, verbose=True)
            exp.log_result("final_eval", {k: v for k, v in final.items() if k != "task_config"})
            for base in (WaypointPolicy(), RandomPolicy(a.seed)):
                b = run_eval(base, cfg, evalset, sim=sim, out_dir=exp.path("evals", f"baseline_{base.name}"), verbose=False)
                exp.log_result(f"baseline_{base.name}", {k: v for k, v in b.items() if k != "task_config"})
                print(f"baseline {base.name}: success {b['success_rate']:.2f}")
            print(f"\nFINAL: success {final['success_rate']:.2f} collision {final['collision_rate']:.2f} "
                  f"final dist {final['mean_final_distance_m']:.2f} m | model: {exp.path('model.zip')}")
        finally:
            eval_pool.close()
    finally:
        try:
            venv.close()
        except Exception:
            pass
        pool.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
