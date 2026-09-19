"""SimulationService: the *environment*. Owns the engine, episodes, agents'
observation/action contracts, environment-driven entities, events, logs."""
from __future__ import annotations

import asyncio
import collections
import datetime as dt
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
from .models import (Action, ActionLimits, ActionSpace, AgentInfo, ArmAction, BodyVelocity, EntityInfo,
                     EntityState, Episode, Event, HoldAction, Metrics, NearbyAgent, Observation,
                     ObservationSpace, Pose, SceneDesc, SensorFrameRef, SimMode, SimStatus, StepResponse,
                     VelocityAction, WaypointAction)
from .observation import NEARBY_RADIUS_M, resolve_profile
from .recording import EpisodeLogger
from .scenario import AgentSpec, CameraSpec, Scenario, list_scenarios

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
        self.agents: dict[str, AgentRuntime] = {}
        self._runtime_spawned: set[str] = set()   # not part of the scenario's initial state
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
            if ("camera" in rt.components or "depth" in rt.components) and rt.spec.camera is None:
                log.warning("agent %s: profile needs camera frames but no camera is configured", rt.spec.id)
        self._trajectories = TrajectoryController(scenario.entities)
        self._open_episode()
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
            self._place_entities(0.0)
            if self.mode == "realtime" and not self.scenario.simulation.start_paused:
                await self.engine.resume()
            self._start_tick()
            self.hub.publish_threadsafe({"type": "reset", "episode": self.episode.model_dump(mode="json")})
        return self.episode, self.observations()

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
        if self.scenario.logging.enabled:
            path = Path(run_dir) / f"episode_{eid}.jsonl"
            self._logger = EpisodeLogger(path, {"episode_id": eid, "scenario": self.scenario.model_dump(mode="json"),
                                                "seed": self.engine.seed, "mode": self.mode})
            self.episode.log_path = str(path)

    def _close_episode(self, status: str) -> None:
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
        # Environment-driven entities are placed kinematically; chunk long steps so they
        # move continuously (every SUBSTEP iterations) instead of teleporting once.
        chunk = SUBSTEP_ITERS if self._trajectories.entities else steps
        done = 0
        while done < steps:
            n = min(chunk, steps - done)
            self._place_entities(t_now + (done + n) * dt)   # applied on the first iteration of this chunk
            if done == 0:
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
        self._step_latency_ms = _ema(self._step_latency_ms, (time.monotonic() - t0) * 1000)
        self._sync_episode()
        return StepResponse(episode=self.episode, status=self.status(), observations=obs,
                            events=new_events, rejected_actions=rejected)

    def status(self) -> SimStatus:
        s = self.engine.status()
        s.mode = self.mode
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
        for aid in self.agents:
            for s in ("camera", "depth"):
                r = self.engine.sensor_rate(aid, s)
                if r:
                    fps[f"{aid}/{s}"] = round(r, 1)
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

    async def spawn(self, entity_id: str, template: str, pose: Pose, params: dict,
                    observation: str | None = None, camera: CameraSpec | None = None) -> None:
        if entity_id in self.agents or any(e.entity_id == entity_id for e in self.engine.list_entities()):
            raise ValueError(f"entity {entity_id!r} already exists")
        is_agent = await self.engine.spawn(entity_id, template, pose, params, camera=camera)
        self._runtime_spawned.add(entity_id)
        if is_agent:
            spec = AgentSpec(id=entity_id, template=template, spawn=pose, params=params,
                             observation=observation or "state", camera=camera)
            comps = resolve_profile(spec.observation, self.scenario.observation_profiles if self.scenario else None)
            self.agents[entity_id] = AgentRuntime(spec, comps)
            self._push_event("agent_spawned", [entity_id], {"template": template})
        self.hub.publish_threadsafe({"type": "scene_changed"})

    async def remove(self, entity_id: str) -> None:
        was_agent = entity_id in self.agents
        self.agents.pop(entity_id, None)          # before the engine lifts it (no out_of_bounds event)
        self._runtime_spawned.discard(entity_id)
        await self.engine.remove(entity_id)
        if was_agent:
            self._push_event("agent_removed", [entity_id], {})
        self.hub.publish_threadsafe({"type": "scene_changed"})

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
        frames = []
        if spec.camera:
            frames.append({"name": "camera", "type": "rgb", "width": spec.camera.width, "height": spec.camera.height,
                           "hfov": spec.camera.hfov, "rate_hz": spec.camera.update_rate})
            if spec.camera.depth:
                frames.append({"name": "depth", "type": "depth", "width": spec.camera.width,
                               "height": spec.camera.height, "hfov": spec.camera.hfov,
                               "rate_hz": spec.camera.update_rate})
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
        for sensor, kind in (("camera", "rgb"), ("depth", "depth")):
            if sensor in comps and sensor in self.engine.sensor_names(agent_id):
                fr = self.engine.frame(agent_id, sensor)
                cam = rt.spec.camera
                obs.frames.append(SensorFrameRef(
                    name=sensor, type=kind, width=fr.width if fr else cam.width, height=fr.height if fr else cam.height,
                    encoding="rgb8" if kind == "rgb" else "depth32f", sim_time=fr.sim_time if fr else None,
                    seq=fr.seq if fr else 0, url=f"/agents/{agent_id}/sensors/{sensor}",
                    stream=f"/ws/sensors/{agent_id}/{sensor}"))
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
            except Exception:
                log.exception("tick failed")

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
        self.hub.publish_threadsafe({
            "type": "state", "sim_time": sim_time, "paused": st.paused, "mode": self.mode,
            "rtf": st.real_time_factor, "iterations": st.iterations,
            "poses": {k: _pose_compact(v) for k, v in poses.items()}, "vel": vel,
        })


def _pose_compact(p: Pose) -> list[float]:
    return [p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]


def _ema(prev: float | None, x: float, a: float = 0.2) -> float:
    return x if prev is None else prev + a * (x - prev)
