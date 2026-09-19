"""SimulationService: the *environment*. Owns the engine, episodes, agents'
observation/action contracts, environment-driven entities, events, logs."""
from __future__ import annotations

import asyncio
import collections
import datetime as dt
import json
import logging
import math
import time
import uuid
from pathlib import Path
from typing import Any

from . import config
from .engine.base import EngineEvent, SimulationEngine
from .envcontrol import TrajectoryController
from .hub import TelemetryHub
from .models import (Action, ActionLimits, ActionSpace, AgentInfo, ArmAction, BodyVelocity, EntityDetail, EntityInfo,
                     EntityState, Episode, Event, HoldAction, Metrics, NearbyAgent, Observation,
                     ObservationSpace, Pose, SceneDesc, SensorFrameRef, SimMode, SimStatus, StepResponse,
                     VelocityAction, WaypointAction)
from .observation import NEARBY_RADIUS_M, resolve_profile
from .recording import EpisodeLogger
from .recorder import Recorder, Snapshot, SnapshotStore
from .scenario import AgentSpec, CameraSpec, Scenario, SensorMount, list_scenarios

log = logging.getLogger(__name__)

WAYPOINT_GAIN = 1.2
REALTIME_TICK_HZ = 20.0
# gz-transport drops messages published before discovery has connected a (re)created
# subscriber. After start/reset we wait this long before accepting actions, and we
# re-send each agent's latched command on every step so a dropped message costs at
# most one step instead of a silently ignored action.
SETTLE_S = 0.4
SUBSTEP_ITERS = 25          # 0.1 s at the default 4 ms step


class ActionError(ValueError):
    """Controlled rejection of an action (bad agent, wrong level, limit violation)."""


class AgentRuntime:
    def __init__(self, spec: AgentSpec, components: list[str]) -> None:
        if spec.limits is None:
            from .engine.gazebo.sdf import DRONE_TYPES
            spec = spec.model_copy(update={"limits": ActionLimits(**DRONE_TYPES.get(spec.drone_type, DRONE_TYPES["standard"])["limits"])})
        self.spec = spec
        self.components = components
        self.last_action: Action | None = None
        self.waypoint: WaypointAction | None = None
        self.control_mode = "idle"
        self.armed = True
        self.out_of_bounds = False


class SimulationService:
    def __init__(self, engine: SimulationEngine, hub: TelemetryHub) -> None:
        self.engine = engine
        self.hub = hub
        self.scenario: Scenario | None = None
        self.episode: Episode | None = None
        self.mode: SimMode = "realtime"
        self.speed: float = 1.0
        self.agents: dict[str, AgentRuntime] = {}
        self._runtime_spawned: set[str] = set()   # not part of the scenario's initial state
        self._runtime_entities: dict[str, Any] = {}  # runtime-spawned non-agent entities with trajectories
        self.overlay: dict | None = None            # last annotation pushed by an external tool
        self.recorder: Recorder | None = None       # per-episode action log (+ optional rows)
        self.snapshots = SnapshotStore(config.RUNS_DIR / "snapshots")
        self._replaying = False
        self.events: collections.deque[Event] = collections.deque(maxlen=5000)
        self._event_seq = 0
        self._events_total = 0
        self._last_step_seq = 0            # events after this seq are 'new' for the next step response
        self._trajectories = TrajectoryController([])
        self._logger: EpisodeLogger | None = None
        self._tick_task: asyncio.Task | None = None
        self._timeout_fired = False
        self._started_wall = time.monotonic()
        self._step_latency_ms: float | None = None
        self._last_broadcast = 0.0
        self._min_interval = 1.0 / config.TELEMETRY_HZ
        self._last_log_sample = 0.0
        self._hz_samples: collections.deque[tuple[float, int]] = collections.deque(maxlen=20)
        self._loop: asyncio.AbstractEventLoop | None = None
        self.engine.on_pose_update(self._on_pose_update)
        self.engine.on_event(self._on_engine_event)

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    # ======================================================================
    # scenarios / episodes
    # ======================================================================
    def scenarios(self) -> list[str]:
        return list_scenarios(config.SCENARIOS_DIR)

    def scenario_path(self, name: str) -> Path:
        return config.SCENARIOS_DIR / f"{name}.yaml"

    async def load_scenario(self, name: str, *, seed: int | None = None, mode: SimMode | None = None) -> Episode:
        return await self.start(Scenario.load(self.scenario_path(name)), seed=seed, mode=mode)

    async def start(self, scenario: Scenario, *, seed: int | None = None, mode: SimMode | None = None) -> Episode:
        await self._stop_tick()
        if self.scenario is not None:
            self._close_episode("terminated")
            await self.engine.shutdown()
        self.mode = mode or scenario.simulation.mode
        self.speed = scenario.simulation.real_time_factor
        # validate profiles before touching the engine
        comps = {a.id: resolve_profile(a.observation, scenario.observation_profiles) for a in scenario.agents}
        if self.mode == "stepped":
            scenario = scenario.model_copy(update={"simulation": scenario.simulation.model_copy(
                update={"start_paused": True, "mode": "stepped"})})
        self.scenario = scenario
        await self.engine.start(scenario, seed=seed)
        self.agents = {a.id: AgentRuntime(a, comps[a.id]) for a in scenario.agents}
        self._runtime_spawned.clear()
        await asyncio.sleep(SETTLE_S)      # let controller systems discover our command publishers
        for rt in self.agents.values():
            if ("camera" in rt.components or "depth" in rt.components) and not rt.spec.has_rendering:
                log.warning("agent %s: profile needs camera frames but no camera is configured", rt.spec.id)
        self._trajectories = TrajectoryController(scenario.entities)
        self._open_episode()
        await self._apply_randomization()
        self._place_entities(0.0)
        self._start_tick()
        self.hub.publish_threadsafe({"type": "scenario_loaded", "scenario": scenario.model_dump(mode="json"),
                                     "episode": self.episode.model_dump(mode="json")})
        return self.episode

    async def shutdown(self) -> None:
        await self._stop_tick()
        self._close_episode("terminated")
        await self.engine.shutdown()
        self.scenario = None
        self.agents.clear()

    async def reset(self, *, seed: int | None = None, scenario: str | None = None,
                    mode: SimMode | None = None) -> tuple[Episode, dict[str, Observation]]:
        if scenario is not None and (self.scenario is None or scenario != self.scenario.name):
            await self.load_scenario(scenario, seed=seed, mode=mode)
        elif self.scenario is None:
            raise RuntimeError("no scenario loaded; pass scenario=")
        elif seed is not None and seed != self.engine.seed:
            # seed is fixed at server launch -> relaunch the same scenario with the new seed
            await self.start(self.scenario, seed=seed, mode=mode or self.mode)
        elif not self.engine.status().running:
            # simulator process is gone (crashed or killed): a reset means "fresh episode", so relaunch
            log.warning("simulator not running; relaunching scenario %s for reset", self.scenario.name)
            await self.start(self.scenario, seed=seed if seed is not None else self.engine.seed, mode=mode or self.mode)
        else:
            await self._stop_tick()
            self._close_episode("terminated" if self.episode and self.episode.status == "running" else
                                (self.episode.status if self.episode else "completed"))
            if mode:
                self.mode = mode
            if self.mode == "stepped":
                await self.engine.pause()
            # Runtime-spawned entities are not part of the episode's initial state. They are
            # removed *before* the rewind: rewinding while they exist crashes gz-sim 10.5
            # (segfault in dartsim GetContactsFromLastStep). Save the scenario to keep them.
            for eid in sorted(self._runtime_spawned):
                self.agents.pop(eid, None)
                if eid in self._runtime_entities:
                    self._trajectories.entities = [e for e in self._trajectories.entities if e.id != eid]
                    self._runtime_entities.pop(eid)
                try:
                    await self.engine.remove(eid)
                except Exception as e:
                    log.warning("could not remove runtime entity %s before reset: %s", eid, e)
            self._runtime_spawned.clear()
            await self.engine.reset()
            await asyncio.sleep(SETTLE_S)  # Gazebo re-creates systems on reset; see SETTLE_S
            for rt in self.agents.values():
                rt.last_action = None
                rt.waypoint = None
                rt.control_mode = "idle"
                rt.armed = True
                rt.out_of_bounds = False
            self._open_episode()
            await self._apply_randomization()
            self._place_entities(0.0)
            if self.mode == "realtime" and not self.scenario.simulation.start_paused:
                await self.engine.resume()
            self._start_tick()
            self.hub.publish_threadsafe({"type": "reset", "episode": self.episode.model_dump(mode="json")})
        return self.episode, self.observations()

    async def _apply_randomization(self) -> None:
        """Seeded initial-condition sampling: agent spawns/yaws, entity positions/routes.
        RNG is seeded by the simulator seed, so the same seed reproduces the same draw."""
        self.randomization: dict[str, Any] = {}
        rz = self.scenario.randomize if self.scenario else None
        if rz is None or (not rz.agents and not rz.entities):
            return
        import random
        rng = random.Random(self.engine.seed)
        # deterministic order: sorted ids
        for aid in sorted(rz.agents):
            spec = rz.agents[aid]
            if aid not in self.agents:
                continue
            draw: dict[str, Any] = {}
            if spec.region:
                r = spec.region
                pos = (rng.uniform(*r.x), rng.uniform(*r.y), rng.uniform(*r.z))
                yaw = rng.uniform(-math.pi, math.pi) if spec.yaw else 0.0
                self.engine.set_pose(aid, Pose(position=type(Pose().position)(x=pos[0], y=pos[1], z=pos[2]),
                                               orientation=type(Pose().orientation)(x=0, y=0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))
                draw["position"] = list(pos); draw["yaw"] = yaw
            self.randomization[aid] = draw
        for eid in sorted(rz.entities):
            spec = rz.entities[eid]
            ent = next((e for e in self._trajectories.entities + (self.scenario.entities if self.scenario else []) if e.id == eid), None)
            draw = {}
            if spec.routes:
                route = rng.choice(spec.routes)
                if ent is not None:
                    ent.trajectory = route
                    if ent not in self._trajectories.entities and route.type != "static":
                        self._trajectories.entities.append(ent)
                draw["route"] = route.model_dump(mode="json")
            if spec.region:
                r = spec.region
                pos = (rng.uniform(*r.x), rng.uniform(*r.y), rng.uniform(*r.z))
                if ent is not None:
                    ent.spawn = Pose(position=type(Pose().position)(x=pos[0], y=pos[1], z=pos[2]))
                    if ent.trajectory.type == "static":
                        self.engine.set_pose(eid, ent.spawn)
                draw["position"] = list(pos)
            self.randomization[eid] = draw
        if rz.time_of_day:
            self.randomization["time_of_day"] = rng.choice(rz.time_of_day)   # informational (world lighting is fixed at load)
        if self.episode:
            self.episode.randomization = self.randomization
        if self._logger:
            self._logger.write({"type": "randomization", "seed": self.engine.seed, "draw": self.randomization})

    def _open_episode(self) -> None:
        assert self.scenario is not None
        eid = dt.datetime.now().strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
        st = self.engine.status()
        self.episode = Episode(
            episode_id=eid, scenario_id=self.scenario.name, seed=self.engine.seed, mode=self.mode,
            start_time=dt.datetime.now().isoformat(timespec="seconds"),
            status="paused" if st.paused else "running", max_sim_time=self.scenario.episode.max_sim_time)
        self.events.clear()
        self._event_seq = 0
        self._last_step_seq = 0
        self._timeout_fired = False
        run_dir = getattr(self.engine, "run_dir", None) or config.RUNS_DIR
        self.recorder = Recorder(Path(run_dir) / "recordings", episode_id=eid, scenario=self.scenario.name,
                                 seed=self.engine.seed, mode=self.mode, step_size=self.scenario.simulation.step_size)
        if self.scenario.logging.enabled:
            path = Path(run_dir) / f"episode_{eid}.jsonl"
            self._logger = EpisodeLogger(path, {"episode_id": eid, "scenario": self.scenario.model_dump(mode="json"),
                                                "seed": self.engine.seed, "mode": self.mode})
            self.episode.log_path = str(path)

    def _close_episode(self, status: str) -> None:
        if self.recorder is not None:
            try:
                self.recorder.finalize(self.engine.status().iterations)
            except Exception:
                pass
            self.recorder = None
        if self.episode and self.episode.status in ("running", "paused", "created"):
            self.episode.status = status  # type: ignore[assignment]
        if self._logger:
            self._logger.close(status)
            self._logger = None

    def _sync_episode(self) -> None:
        if not self.episode:
            return
        st = self.engine.status()
        self.episode.sim_time = st.sim_time
        self.episode.iterations = st.iterations
        if self.episode.status in ("running", "paused"):
            if not st.running:                      # simulator process died
                self.episode.status = "failed"
                self._push_event("simulator_crashed", [], {"log": getattr(self.engine, "crash_log", "")})
                if self._logger:
                    self._logger.close("failed")
                    self._logger = None
            else:
                self.episode.status = "paused" if st.paused else "running"

    # ======================================================================
    # lifecycle controls
    # ======================================================================
    async def pause(self) -> SimStatus:
        await self.engine.pause()
        return self.status()

    async def resume(self) -> SimStatus:
        if self.mode == "stepped":
            raise RuntimeError("simulation is in stepped mode; switch mode to 'realtime' to free-run")
        if self.episode and self.episode.status in ("completed", "terminated", "failed"):
            raise RuntimeError(f"episode is {self.episode.status}; reset first")
        await self.engine.resume()
        return self.status()

    async def set_speed(self, real_time_factor: float) -> SimStatus:
        if real_time_factor < 0:
            raise ValueError("speed must be >= 0 (0 = unlimited)")
        await asyncio.get_running_loop().run_in_executor(None, self.engine.set_speed, real_time_factor)
        self.speed = real_time_factor
        return self.status()

    async def set_mode(self, mode: SimMode) -> SimStatus:
        self.mode = mode
        if self.episode:
            self.episode.mode = mode
        if mode == "stepped":
            await self.engine.pause()
        return self.status()

    async def step(self, steps: int = 1, actions: dict[str, Action] | None = None,
                   observe: bool = True) -> StepResponse:
        if self.scenario is None:
            raise RuntimeError("no scenario loaded")
        if self.episode and self.episode.status in ("completed", "terminated", "failed"):
            raise RuntimeError(f"episode is {self.episode.status}; reset first")
        t0 = time.monotonic()
        rejected: dict[str, str] = {}
        for aid, act in (actions or {}).items():
            try:
                self._apply_action(aid, act)
            except ActionError as e:
                rejected[aid] = str(e)
        seq_before = self._last_step_seq   # events since the previous step response
        dt = self.scenario.simulation.step_size
        t_now = self.engine.status().sim_time
        # Long steps are chunked (SUBSTEP_ITERS = 0.1 s) whenever something inside the environment
        # needs re-evaluating during the step: kinematic entities move continuously instead of
        # teleporting once, and waypoint controllers close the loop instead of flying open-loop
        # for the whole step. Chunking is deterministic, so replay/snapshots are unaffected.
        chunk = SUBSTEP_ITERS if self._needs_substeps() else steps
        done = 0
        while done < steps:
            n = min(chunk, steps - done)
            self._place_entities(t_now + (done + n) * dt)   # applied on the first iteration of this chunk
            self._run_waypoint_controllers()
            await self.engine.step(n)
            done += n
        if self.episode:
            self.episode.step_count += 1
        self._check_bounds_and_timeout()
        new_events = [e for e in self.events if e.seq > seq_before]
        self._last_step_seq = self._event_seq
        obs = self.observations() if observe else {}
        self._log_sample(new_events)
        self._record_row(actions or {}, new_events, obs)
        self._step_latency_ms = _ema(self._step_latency_ms, (time.monotonic() - t0) * 1000)
        self._sync_episode()
        return StepResponse(episode=self.episode, status=self.status(), observations=obs,
                            events=new_events, rejected_actions=rejected)

    def status(self) -> SimStatus:
        s = self.engine.status()
        s.mode = self.mode
        s.speed = self.speed
        if self.scenario:
            s.scenario = self.scenario.name
            s.seed = self.engine.seed
            s.step_size = self.scenario.simulation.step_size
        self._sync_episode()
        s.episode = self.episode
        return s

    def metrics(self) -> Metrics:
        st = self.engine.status()
        now = time.monotonic()
        self._hz_samples.append((now, st.iterations))
        hz = 0.0
        if len(self._hz_samples) >= 2:
            (t0, i0), (t1, i1) = self._hz_samples[0], self._hz_samples[-1]
            hz = (i1 - i0) / (t1 - t0) if t1 > t0 else 0.0
        fps = {}
        for aid, rt in self.agents.items():
            for m in rt.spec.camera_mounts:
                r = self.engine.sensor_rate(aid, m.name)
                if r:
                    fps[f"{aid}/{m.name}"] = round(r, 1)
        return Metrics(real_time_factor=st.real_time_factor, sim_hz=hz, step_size=st.step_size or
                       (self.scenario.simulation.step_size if self.scenario else None), sim_time=st.sim_time,
                       entity_count=len(self.engine.list_entities()), agent_count=len(self.agents),
                       telemetry_clients=self.hub.client_count, sensor_clients=self.hub.sensor_clients,
                       api_step_latency_ms=self._step_latency_ms, sensor_fps=fps,
                       events_total=self._events_total, uptime_s=now - self._started_wall)

    # ======================================================================
    # world / entities
    # ======================================================================
    async def scene(self) -> SceneDesc:
        return await self.engine.scene()

    def entities(self) -> list[EntityInfo]:
        return self.engine.list_entities()

    def entity_state(self, entity_id: str) -> EntityState | None:
        return self.engine.entity_state(entity_id)

    async def entity_detail(self, entity_id: str) -> EntityDetail | None:
        infos = {e.entity_id: e for e in self.engine.list_entities()}
        info = infos.get(entity_id)
        if info is None:
            return None
        scene = await self.engine.scene()
        model = next((m for m in scene.models if m.entity_id == entity_id), None)
        dims = _bbox(model) if model else None
        rt = self.agents.get(entity_id)
        ent = next((e for e in (self.scenario.entities if self.scenario else []) if e.id == entity_id), None) \
            or self._runtime_entities.get(entity_id)
        category = info.category
        detail = EntityDetail(entity_id=entity_id, kind=info.kind, category=category, is_agent=info.is_agent,
                              is_static=model.is_static if model else False, dimensions=dims,
                              collision=bool(model and any(l.collisions or l.visuals for l in model.links)),
                              state=self.engine.entity_state(entity_id))
        if rt is not None:
            from .engine.gazebo.sdf import DRONE_TYPES
            T = DRONE_TYPES.get(rt.spec.drone_type, {})
            detail.template = rt.spec.template
            detail.type_label = T.get("label", "Quadrotor")
            detail.control = rt.control_mode
            detail.sensors = self.engine.sensor_names(entity_id)
            detail.sensor_mounts = [m.model_dump() for m in rt.spec.sensors] or \
                                   [m.model_dump() for m in rt.spec.camera_mounts]
            detail.physical = {"drone_type": rt.spec.drone_type, "mass_kg": T.get("mass"),
                               "limits": rt.spec.limits.model_dump() if rt.spec.limits else None,
                               "observation_profile": rt.spec.observation, "armed": rt.armed}
        elif ent is not None:
            detail.template = ent.template
            detail.type_label = {"vehicle": "Ground vehicle", "target": "Target marker", "platform": "Moving platform",
                                 "beacon": "Rotating beacon"}.get(ent.template, ent.template)
            tr = ent.trajectory
            detail.trajectory = {"circle": f"circle r={tr.radius} m @ {tr.speed} m/s", "line": f"line @ {tr.speed} m/s{' (loop)' if tr.loop else ''}",
                                 "waypoints": f"{len(tr.waypoints)} waypoints @ {tr.speed} m/s", "rotate": f"rotate {tr.yaw_rate} rad/s",
                                 "static": None}[tr.type]
            detail.physical = dict(ent.params)
        else:
            detail.type_label = {"building": "Building", "obstacle": "Obstacle", "static": "Terrain"}.get(info.kind, info.kind)
        return detail

    async def spawn(self, entity_id: str, template: str, pose: Pose, params: dict,
                    observation: str | None = None, camera: CameraSpec | None = None,
                    drone_type: str = "standard", sensors: list[SensorMount] | None = None,
                    trajectory=None, control: str = "waypoint") -> None:
        if entity_id in self.agents or any(e.entity_id == entity_id for e in self.engine.list_entities()):
            raise ValueError(f"entity {entity_id!r} already exists")
        sensors = list(sensors or [])
        is_agent = await self.engine.spawn(entity_id, template, pose, params, camera=camera, drone_type=drone_type,
                                           mounts=[m for m in sensors if m.type in ("camera", "depth")])
        self._runtime_spawned.add(entity_id)
        if self.recorder is not None and not self._replaying:
            st = self.engine.status()
            self.recorder.log_op(st.iterations, st.sim_time, "spawn", {
                "entity_id": entity_id, "template": template, "pose": pose.model_dump(mode="json"), "params": params,
                "observation": observation, "camera": camera.model_dump(mode="json") if camera else None,
                "drone_type": drone_type, "sensors": [m.model_dump(mode="json") for m in sensors],
                "trajectory": trajectory, "control": control})
        if not is_agent and trajectory is not None:
            from .scenario import EntitySpec, TrajectorySpec
            spec_e = EntitySpec(id=entity_id, template=template, spawn=pose, trajectory=TrajectorySpec.model_validate(trajectory), params=params)
            self._trajectories.entities.append(spec_e)
            self._runtime_entities[entity_id] = spec_e
        if is_agent:
            spec = AgentSpec(id=entity_id, template=template, spawn=pose, params=params, drone_type=drone_type,
                             observation=observation or "state", camera=camera, sensors=sensors, control=control)
            comps = resolve_profile(spec.observation, self.scenario.observation_profiles if self.scenario else None)
            self.agents[entity_id] = AgentRuntime(spec, comps)
            self._push_event("agent_spawned", [entity_id], {"template": template})
        self.hub.publish_threadsafe({"type": "scene_changed"})

    async def remove(self, entity_id: str) -> None:
        was_agent = entity_id in self.agents
        if self.recorder is not None and not self._replaying:
            st = self.engine.status()
            self.recorder.log_op(st.iterations, st.sim_time, "remove", {"entity_id": entity_id})
        self.agents.pop(entity_id, None)          # before the engine lifts it (no out_of_bounds event)
        self._runtime_spawned.discard(entity_id)
        if entity_id in self._runtime_entities:
            self._trajectories.entities = [e for e in self._trajectories.entities if e.id != entity_id]
            self._runtime_entities.pop(entity_id)
        await self.engine.remove(entity_id)
        if was_agent:
            self._push_event("agent_removed", [entity_id], {})
        self.hub.publish_threadsafe({"type": "scene_changed"})

    def teleport(self, entity_id: str, pose: Pose) -> None:
        """Kinematically place any entity (used by task wrappers to randomise starts/targets)."""
        if not any(e.entity_id == entity_id for e in self.engine.list_entities()):
            raise ValueError(f"unknown entity {entity_id!r}")
        self.engine.set_pose(entity_id, pose)
        rt = self.agents.get(entity_id)
        if rt is not None:
            rt.waypoint = None          # a teleported agent drops any pending waypoint
        if self.recorder is not None and not self._replaying:
            st = self.engine.status()
            self.recorder.log_op(st.iterations, st.sim_time, "teleport", {"entity_id": entity_id, "pose": pose.model_dump(mode="json")})

    def _place_entities(self, sim_time: float) -> None:
        for eid, pose in self._trajectories.targets(sim_time).items():
            try:
                self.engine.set_pose(eid, pose)
            except Exception as e:  # engine not ready yet
                log.debug("set_pose %s failed: %s", eid, e)

    # ======================================================================
    # agents: contracts, observations, actions
    # ======================================================================
    def agent_ids(self) -> list[str]:
        return list(self.agents)

    def agent_info(self, agent_id: str) -> AgentInfo | None:
        rt = self.agents.get(agent_id)
        if rt is None:
            return None
        spec = rt.spec
        frames = [{"name": m.name, "type": "rgb" if m.type == "camera" else "depth", "width": m.params.get("width", 320),
                   "height": m.params.get("height", 240), "hfov": m.params.get("hfov", 1.396),
                   "rate_hz": m.params.get("update_rate", 15), "pose": m.pose} for m in spec.camera_mounts]
        types = ["velocity", "hold", "arm"] + (["waypoint"] if spec.control == "waypoint" else [])
        fields = {
            "velocity": {"vx": "m/s", "vy": "m/s", "vz": "m/s", "yaw_rate": "rad/s", "frame": "body|world"},
            "hold": {}, "arm": {"armed": "bool"},
        }
        if spec.control == "waypoint":
            fields["waypoint"] = {"x": "m", "y": "m", "z": "m", "yaw": "rad|null", "speed": "m/s", "tolerance": "m"}
        return AgentInfo(
            agent_id=agent_id, agent_type=spec.type, template=spec.template,
            observation_space=ObservationSpace(profile=spec.observation, components=rt.components, frames=frames),
            action_space=ActionSpace(level=spec.control, types=types, fields=fields, limits=spec.limits),
            sensors=self.engine.sensor_names(agent_id), control_mode=rt.control_mode,
            state=self.engine.entity_state(agent_id))

    def observation(self, agent_id: str) -> Observation | None:
        rt = self.agents.get(agent_id)
        if rt is None:
            return None
        st = self.engine.entity_state(agent_id)
        if st is None:
            return None
        status = self.engine.status()
        comps = rt.components
        obs = Observation(agent_id=agent_id, sim_time=st.sim_time, iteration=status.iterations,
                          profile=rt.spec.observation, grounded=self.engine.grounded(agent_id),
                          armed=rt.armed, control_mode=rt.control_mode)
        if "state" in comps:
            obs.state = st
        if "swarm_state" in comps:
            obs.swarm_state = [s for a in self.agents if (s := self.engine.entity_state(a)) is not None]
        if "velocity" in comps:
            bv = self.engine.body_velocity(agent_id)
            obs.velocity = BodyVelocity(linear=bv[0], angular=bv[1]) if bv else BodyVelocity()
        if "imu" in comps:
            obs.imu = self.engine.imu(agent_id)
        if "gps" in comps:
            obs.gps = self.engine.gps(agent_id)
        if "nearby_agents" in comps:
            obs.nearby_agents = self._nearby(agent_id, st)
        for m in rt.spec.camera_mounts:
            comp = "camera" if m.type == "camera" else "depth"
            if comp not in comps or m.name not in self.engine.sensor_names(agent_id):
                continue
            fr = self.engine.frame(agent_id, m.name)
            kind = "rgb" if m.type == "camera" else "depth"
            obs.frames.append(SensorFrameRef(
                name=m.name, type=kind, width=fr.width if fr else m.params.get("width", 320),
                height=fr.height if fr else m.params.get("height", 240),
                encoding="rgb8" if kind == "rgb" else "depth32f", sim_time=fr.sim_time if fr else None,
                seq=fr.seq if fr else 0, url=f"/agents/{agent_id}/sensors/{m.name}",
                stream=f"/ws/sensors/{agent_id}/{m.name}"))
        return obs

    def observations(self) -> dict[str, Observation]:
        return {a: o for a in self.agents if (o := self.observation(a)) is not None}

    def _nearby(self, agent_id: str, me: EntityState) -> list[NearbyAgent]:
        out = []
        p = me.pose.position
        for other in self.agents:
            if other == agent_id:
                continue
            st = self.engine.entity_state(other)
            if st is None:
                continue
            q = st.pose.position
            d = math.sqrt((q.x - p.x) ** 2 + (q.y - p.y) ** 2 + (q.z - p.z) ** 2)
            if d <= NEARBY_RADIUS_M:
                out.append(NearbyAgent(agent_id=other, distance=d, velocity=st.linear_velocity,
                                       relative_position=type(p)(x=q.x - p.x, y=q.y - p.y, z=q.z - p.z)))
        return sorted(out, key=lambda n: n.distance)

    def validate_action(self, agent_id: str, action: Action) -> AgentRuntime:
        rt = self.agents.get(agent_id)
        if rt is None:
            raise ActionError(f"unknown agent {agent_id!r}")
        if isinstance(action, WaypointAction) and rt.spec.control != "waypoint":
            raise ActionError(f"agent {agent_id!r} accepts velocity-level actions only (control={rt.spec.control})")
        lim: ActionLimits = rt.spec.limits
        if isinstance(action, (VelocityAction, WaypointAction)):
            v = action.violations(lim)
            if v:
                raise ActionError("action exceeds limits: " + "; ".join(v))
        return rt

    def _apply_action(self, agent_id: str, action: Action) -> None:
        rt = self.validate_action(agent_id, action)
        if self.recorder is not None and not self._replaying:
            st = self.engine.status()
            self.recorder.log_action(st.iterations, st.sim_time, agent_id, action)
        if isinstance(action, VelocityAction):
            rt.waypoint = None
            rt.control_mode = "velocity"
            self.engine.send_velocity(agent_id, action)
        elif isinstance(action, HoldAction):
            rt.waypoint = None
            rt.control_mode = "hold"
            self.engine.send_velocity(agent_id, VelocityAction())
        elif isinstance(action, WaypointAction):
            rt.waypoint = action
            rt.control_mode = "waypoint"
            self._drive_waypoint(agent_id, rt)
        elif isinstance(action, ArmAction):
            rt.armed = action.armed
            self.engine.send_arm(agent_id, action.armed)
        rt.last_action = action

    async def send_action(self, agent_id: str, action: Action) -> None:
        self._apply_action(agent_id, action)

    def _needs_substeps(self) -> bool:
        return bool(self._trajectories.entities) or any(rt.waypoint is not None for rt in self.agents.values())

    def _run_waypoint_controllers(self) -> None:
        """Per-step re-issue of every agent's current command (latching)."""
        for aid, rt in self.agents.items():
            if rt.waypoint is not None:
                self._drive_waypoint(aid, rt)
            elif isinstance(rt.last_action, VelocityAction):
                self.engine.send_velocity(aid, rt.last_action)
            elif isinstance(rt.last_action, HoldAction):
                self.engine.send_velocity(aid, VelocityAction())

    def _drive_waypoint(self, agent_id: str, rt: AgentRuntime) -> None:
        """Environment-side P controller: waypoint -> world-frame velocity setpoint."""
        wp = rt.waypoint
        st = self.engine.entity_state(agent_id)
        if wp is None or st is None:
            return
        p = st.pose.position
        dx, dy, dz = wp.x - p.x, wp.y - p.y, wp.z - p.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        yaw_rate = 0.0
        if wp.yaw is not None:
            q = st.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            err = math.atan2(math.sin(wp.yaw - yaw), math.cos(wp.yaw - yaw))
            yaw_rate = max(-rt.spec.limits.max_yaw_rate, min(rt.spec.limits.max_yaw_rate, 1.5 * err))
        if dist < wp.tolerance:
            cmd = VelocityAction(frame="world", yaw_rate=yaw_rate)
        else:
            vx, vy, vz = WAYPOINT_GAIN * dx, WAYPOINT_GAIN * dy, WAYPOINT_GAIN * dz
            n = math.sqrt(vx * vx + vy * vy + vz * vz)
            speed = min(wp.speed, rt.spec.limits.max_speed_xy)
            if n > speed:
                vx, vy, vz = vx * speed / n, vy * speed / n, vz * speed / n
            cmd = VelocityAction(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate, frame="world").clamp(rt.spec.limits)
        self.engine.send_velocity(agent_id, cmd)

    # ======================================================================
    # world state, recording, snapshots, replay
    # ======================================================================
    def world_state(self) -> dict[str, Any]:
        st = self.status()
        return {
            "sim_time": st.sim_time, "iteration": st.iterations, "paused": st.paused, "mode": self.mode,
            "episode": self.episode.model_dump(mode="json") if self.episode else None,
            "entities": [s.model_dump(mode="json") for e in self.engine.list_entities()
                         if (s := self.engine.entity_state(e.entity_id))],
            "agents": {aid: {"control_mode": rt.control_mode, "armed": rt.armed,
                             "last_action": rt.last_action.model_dump(mode="json") if rt.last_action else None,
                             "waypoint": rt.waypoint.model_dump(mode="json") if rt.waypoint else None}
                       for aid, rt in self.agents.items()},
            "events_total": self._event_seq,
            "recording": self.recorder.meta.__dict__ if self.recorder else None,
            "randomization": getattr(self, "randomization", {}),
        }

    def recording_start(self, *, observations: bool = True, states: bool = True, frames: bool = False) -> dict[str, Any]:
        if self.recorder is None:
            raise RuntimeError("no episode; load a scenario first")
        self.recorder.start_rows(observations=observations, states=states, frames=frames)
        return self.recorder.meta.__dict__

    def recording_stop(self) -> dict[str, Any]:
        if self.recorder is None:
            raise RuntimeError("no episode")
        self.recorder.stop_rows()
        return self.recorder.meta.__dict__

    def recordings(self) -> list[dict[str, Any]]:
        out = []
        for meta in sorted(config.RUNS_DIR.glob("*/recordings/*/meta.json")):
            try:
                d = json.loads(meta.read_text()); d["dir"] = str(meta.parent); out.append(d)
            except Exception:
                continue
        return out

    def recording_dir(self, recording_id: str) -> Path:
        if self.recorder is not None and self.recorder.meta.recording_id == recording_id:
            return self.recorder.dir
        hits = list(config.RUNS_DIR.glob(f"*/recordings/{recording_id}"))
        if not hits:
            raise FileNotFoundError(f"recording {recording_id!r} not found")
        return hits[0]

    def snapshot(self, name: str | None = None) -> Snapshot:
        if self.recorder is None or self.episode is None or self.scenario is None:
            raise RuntimeError("no episode; load a scenario first")
        st = self.status()
        ws = self.world_state()
        snap = Snapshot(
            snapshot_id=SnapshotStore.new_id(name), name=name or "snapshot", created=time.time(),
            episode_id=self.episode.episode_id, scenario=self.scenario.name, seed=self.engine.seed, mode=self.mode,
            sim_time=st.sim_time, iteration=st.iterations, step_count=self.episode.step_count,
            recording_dir=str(self.recorder.dir), action_index=len(self.recorder.actions),
            entities={e["entity_id"]: e for e in ws["entities"]}, agents=ws["agents"], events_total=self._event_seq)
        # persist a copy of the action log alongside the snapshot so it survives later episodes
        self.snapshots.save(snap)
        actions_copy = self.snapshots.dir / f"{snap.snapshot_id}.actions.jsonl"
        actions_copy.write_text("".join(json.dumps(a.__dict__, separators=(",", ":")) + "\n"
                                        for a in self.recorder.actions[:snap.action_index]))
        return snap

    async def restore(self, snapshot_id: str) -> dict[str, Any]:
        """Replay-based restore: reset(scenario, seed) then re-apply the recorded actions up to the
        snapshot, stepping the exact iteration counts between them. Returns the divergence between
        the restored entity states and the captured ones (expected ~0 for stepped recordings)."""
        snap = self.snapshots.load(snapshot_id)
        actions_path = self.snapshots.dir / f"{snapshot_id}.actions.jsonl"
        from .recorder import ActionRecord
        actions = [ActionRecord(**json.loads(l)) for l in actions_path.open() if l.strip()] if actions_path.exists() else []
        await self._replay_actions(snap.scenario, snap.seed, actions, snap.iteration)
        ws = self.world_state()
        div = _divergence(snap.entities, {e["entity_id"]: e for e in ws["entities"]})
        self.hub.publish_threadsafe({"type": "restored", "snapshot": snapshot_id, "divergence": div})
        return {"snapshot": snap.to_json(), "divergence": div, "state": ws}

    async def replay(self, recording_id: str, *, until_iteration: int | None = None) -> dict[str, Any]:
        """Re-run a recorded episode deterministically (stepped). Emits events and telemetry as it goes."""
        d = self.recording_dir(recording_id)
        is_current = self.recorder is not None and self.recorder.meta.recording_id == recording_id
        if is_current:
            self.recorder.meta.final_iteration = self.engine.status().iterations   # live episode: not finalized yet
        meta = Recorder.load_meta(d) if not is_current else self.recorder.meta
        actions = Recorder.load_actions(d) if not is_current else list(self.recorder.actions)
        target = until_iteration if until_iteration is not None else (
            meta.final_iteration or (actions[-1].iteration if actions else 0))
        await self._replay_actions(meta.scenario, meta.seed, actions, target)
        return {"recording": meta.__dict__, "replayed_to_iteration": target, "state": self.world_state()}

    async def _replay_actions(self, scenario: str, seed: int, actions, until_iteration: int) -> None:
        """Reset and re-apply a log. The re-applied operations/actions are logged into the *new*
        episode's recording, so a restored or replayed episode is self-contained: replaying its
        own recording from a bare reset reproduces it (the prefix is part of its history)."""
        self._replaying = False
        try:
            await self.reset(seed=seed, scenario=scenario, mode="stepped")
            if self.scenario is None or self.scenario.name != scenario or self.engine.seed != seed:
                await self.load_scenario(scenario, seed=seed, mode="stepped")
            cur = 0
            from .models import ActionEnvelope
            for rec in actions:
                if rec.iteration > until_iteration:
                    break
                if rec.iteration > cur:
                    await self._raw_step(rec.iteration - cur); cur = rec.iteration
                if rec.op:
                    await self._replay_op(rec.op, rec.args or {})
                elif rec.agent_id and rec.action:
                    self._apply_action(rec.agent_id, ActionEnvelope.model_validate({"action": rec.action}).action)
            if until_iteration > cur:
                await self._raw_step(until_iteration - cur)
        finally:
            self._replaying = False

    async def _replay_op(self, op: str, a: dict[str, Any]) -> None:
        if op == "spawn":
            cam = CameraSpec.model_validate(a["camera"]) if a.get("camera") else None
            await self.spawn(a["entity_id"], a["template"], Pose.model_validate(a["pose"]), a.get("params") or {},
                             observation=a.get("observation"), camera=cam, drone_type=a.get("drone_type", "standard"),
                             sensors=[SensorMount.model_validate(m) for m in a.get("sensors") or []],
                             trajectory=a.get("trajectory"), control=a.get("control", "waypoint"))
            await asyncio.sleep(SETTLE_S)      # the new controller must discover our publishers, as at start
        elif op == "remove":
            await self.remove(a["entity_id"])
        elif op == "teleport":
            self.teleport(a["entity_id"], Pose.model_validate(a["pose"]))

    async def _raw_step(self, n: int) -> None:
        """Advance n iterations honouring environment entities and waypoint controllers, without
        recording (replay must not re-log the actions it replays)."""
        dt = self.scenario.simulation.step_size
        t_now = self.engine.status().sim_time
        chunk = SUBSTEP_ITERS if self._needs_substeps() else n
        done = 0
        while done < n:
            k = min(chunk, n - done)
            self._place_entities(t_now + (done + k) * dt)
            self._run_waypoint_controllers()
            await self.engine.step(k)
            done += k
        if self.episode:
            self.episode.step_count += 1
        self._check_bounds_and_timeout()

    # ======================================================================
    # events
    # ======================================================================
    def _push_event(self, name: str, entities: list[str], data: dict[str, Any], sim_time: float | None = None) -> Event:
        self._event_seq += 1
        self._events_total += 1
        ev = Event(seq=self._event_seq, event=name, entities=entities, data=data,
                   sim_time=self.engine.status().sim_time if sim_time is None else sim_time,
                   episode_id=self.episode.episode_id if self.episode else None)
        self.events.append(ev)
        if self._logger:
            self._logger.event(ev.model_dump(mode="json"))
        self.hub.publish_threadsafe({"type": "event", **ev.model_dump(mode="json")})
        if self.scenario and name in self.scenario.episode.terminate_on and self.episode \
                and self.episode.status in ("running", "paused"):
            self._end_episode("terminated", reason=name)
        return ev

    def _end_episode(self, status: str, reason: str) -> None:
        if self.episode:
            self.episode.status = status  # type: ignore[assignment]
        self._push_event("episode_end", [], {"status": status, "reason": reason})
        if self._logger:
            self._logger.close(status)
            self._logger = None
        if self._loop:
            self._loop.call_soon_threadsafe(lambda: asyncio.ensure_future(self.engine.pause()))

    def _on_engine_event(self, ev: EngineEvent) -> None:
        name, sim_time, entities, data = ev
        self._push_event(name, entities, data, sim_time)

    def events_since(self, seq: int) -> list[Event]:
        return [e for e in self.events if e.seq > seq]

    def _check_bounds_and_timeout(self) -> None:
        if not self.scenario:
            return
        b = self.scenario.world.bounds
        if b:
            for aid, rt in self.agents.items():
                st = self.engine.entity_state(aid)
                if st is None:
                    continue
                p = st.pose.position
                outside = any(not (lo <= getattr(p, ax) <= hi) for ax, (lo, hi) in b.items())
                if outside and not rt.out_of_bounds:
                    self._push_event("out_of_bounds", [aid], {"position": p.model_dump()}, st.sim_time)
                elif not outside and rt.out_of_bounds:
                    self._push_event("in_bounds", [aid], {}, st.sim_time)
                rt.out_of_bounds = outside
        mt = self.scenario.episode.max_sim_time
        if mt is not None and not self._timeout_fired and self.engine.status().sim_time >= mt:
            self._timeout_fired = True
            self._push_event("timeout", list(self.agents), {"max_sim_time": mt})
            if self.episode and self.episode.status in ("running", "paused"):
                self._end_episode("completed", reason="timeout")

    # ======================================================================
    # realtime tick: waypoint controllers, trajectories, bounds, logging
    # ======================================================================
    def _start_tick(self) -> None:
        if self._loop and (self._tick_task is None or self._tick_task.done()):
            self._tick_task = self._loop.create_task(self._tick())

    async def _stop_tick(self) -> None:
        if self._tick_task:
            self._tick_task.cancel()
            try:
                await self._tick_task
            except (asyncio.CancelledError, Exception):
                pass
            self._tick_task = None

    async def _tick(self) -> None:
        period = 1.0 / REALTIME_TICK_HZ
        seq_logged = 0
        while True:
            await asyncio.sleep(period)
            try:
                st = self.engine.status()
                self._hz_samples.append((time.monotonic(), st.iterations))
                if not st.running or st.paused or self.mode == "stepped":
                    continue   # in stepped mode everything happens inside step(); never race it
                # free-running: drive environment entities and waypoint controllers
                self._place_entities(st.sim_time + period * max(st.real_time_factor, 0.5))
                self._run_waypoint_controllers()
                self._check_bounds_and_timeout()
                if self.scenario and self._logger and self.scenario.logging.realtime_hz > 0:
                    now = time.monotonic()
                    if now - self._last_log_sample >= 1.0 / self.scenario.logging.realtime_hz:
                        self._last_log_sample = now
                        new = [e for e in self.events if e.seq > seq_logged]
                        seq_logged = self._event_seq
                        self._log_sample(new)
                        self._record_row({}, new)
            except Exception:
                log.exception("tick failed")

    def _record_row(self, actions: dict[str, Action], events: list[Event], obs: dict[str, Observation] | None = None) -> None:
        if self.recorder is None or not self.recorder.rows_enabled:
            return
        st = self.engine.status()
        states = {e.entity_id: s.model_dump(mode="json") for e in self.engine.list_entities()
                  if e.kind in ("drone", "vehicle", "target", "dynamic") and (s := self.engine.entity_state(e.entity_id))}
        if obs is None and self.recorder.meta.observations:
            obs = self.observations()
        self.recorder.row(iteration=st.iterations, sim_time=st.sim_time,
                          actions={k: v.model_dump(mode="json") for k, v in actions.items()}, states=states,
                          observations={k: v.model_dump(mode="json") for k, v in (obs or {}).items()},
                          events=[e.model_dump(mode="json") for e in events])

    def _log_sample(self, events: list[Event]) -> None:
        if not self._logger:
            return
        st = self.engine.status()
        agents = {}
        for aid, rt in self.agents.items():
            s = self.engine.entity_state(aid)
            agents[aid] = {"state": s.model_dump(mode="json") if s else None,
                           "action": rt.last_action.model_dump(mode="json") if rt.last_action else None,
                           "control_mode": rt.control_mode}
        self._logger.sample(sim_time=st.sim_time, iteration=st.iterations,
                            step=self.episode.step_count if self.episode else 0, agents=agents,
                            events=[e.model_dump(mode="json") for e in events])

    # ======================================================================
    # telemetry
    # ======================================================================
    def _on_pose_update(self, sim_time: float, poses: dict[str, Pose]) -> None:
        now = time.monotonic()
        if now - self._last_broadcast < self._min_interval:
            return
        self._last_broadcast = now
        st = self.engine.status()
        vel = {}
        for aid in self.agents:
            s = self.engine.entity_state(aid)
            if s:
                vel[aid] = [round(s.linear_velocity.x, 3), round(s.linear_velocity.y, 3), round(s.linear_velocity.z, 3)]
        wps = {aid: [rt.waypoint.x, rt.waypoint.y, rt.waypoint.z] for aid, rt in self.agents.items() if rt.waypoint}
        self.hub.publish_threadsafe({
            "type": "state", "sim_time": sim_time, "paused": st.paused, "mode": self.mode,
            "rtf": st.real_time_factor, "iterations": st.iterations,
            "poses": {k: _pose_compact(v) for k, v in poses.items()}, "vel": vel, "wp": wps,
        })


def _pose_compact(p: Pose) -> list[float]:
    return [p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]


def _ema(prev: float | None, x: float, a: float = 0.2) -> float:
    return x if prev is None else prev + a * (x - prev)


def _divergence(a: dict[str, dict], b: dict[str, dict]) -> dict[str, Any]:
    """Max position / velocity difference between two entity-state dicts (metres, m/s)."""
    max_pos = max_vel = 0.0
    per = {}
    for eid, sa in a.items():
        sb = b.get(eid)
        if sb is None:
            continue
        pa, pb = sa["pose"]["position"], sb["pose"]["position"]
        va, vb = sa["linear_velocity"], sb["linear_velocity"]
        dp = math.sqrt(sum((pa[k] - pb[k]) ** 2 for k in "xyz"))
        dv = math.sqrt(sum((va[k] - vb[k]) ** 2 for k in "xyz"))
        per[eid] = {"position": dp, "velocity": dv}
        max_pos, max_vel = max(max_pos, dp), max(max_vel, dv)
    return {"max_position_m": max_pos, "max_velocity_mps": max_vel, "entities": per}


def _bbox(model):
    """Axis-aligned extent of a model's visuals (ignores rotation of visuals; fine for boxes/cylinders)."""
    from .models import Vec3
    lo = [float("inf")] * 3; hi = [float("-inf")] * 3
    found = False
    for link in model.links:
        for v in link.visuals:
            g = v.geometry
            if g.type == "box" and g.size:
                half = (g.size.x / 2, g.size.y / 2, g.size.z / 2)
            elif g.type == "cylinder" and g.radius is not None:
                half = (g.radius, g.radius, (g.length or 0) / 2)
            elif g.type == "sphere" and g.radius is not None:
                half = (g.radius,) * 3
            elif g.type == "plane" and g.size:
                half = (g.size.x / 2, g.size.y / 2, 0.0)
            else:
                continue
            c = (link.pose.position.x + v.pose.position.x, link.pose.position.y + v.pose.position.y,
                 link.pose.position.z + v.pose.position.z)
            for i in range(3):
                lo[i] = min(lo[i], c[i] - half[i]); hi[i] = max(hi[i], c[i] + half[i])
            found = True
    return Vec3(x=hi[0] - lo[0], y=hi[1] - lo[1], z=hi[2] - lo[2]) if found else None


def _category(kind: str, name: str, is_static: bool) -> str:
    if kind == "drone":
        return "drone"
    if kind in ("vehicle", "target"):
        return kind
    if kind == "dynamic":
        return "dynamic"
    if name == "ground" or kind == "static":
        return "terrain"
    if kind == "building":
        return "building"
    if any(s in name for s in ("pad", "tower", "mast", "road", "bridge", "platform", "light", "fence")):
        return "infrastructure"
    return "obstacle"
