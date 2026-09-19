"""GazeboEngine: SimulationEngine on top of a headless `gz sim -s` server driven
over gz-transport (Gazebo Sim Jetty / gz-sim 10).

Topics/services used (world W, agent A):
  /world/W/control              service  WorldControl -> Boolean   (pause/step/reset)
  /world/W/stats                topic    WorldStatistics
  /world/W/scene/info           service  Empty -> Scene            (entity tree + geometry)
  /world/W/dynamic_pose/info    topic    Pose_V                    (moving entities @60Hz)
  /world/W/create | /remove     service  EntityFactory | Entity -> Boolean
  /world/W/set_pose             service  Pose -> Boolean           (kinematic placement)
  /model/A/odometry             topic    Odometry                  (OdometryPublisher system)
  /A/imu  /A/navsat  /A/contacts  /A/camera  /A/depth              (sensors)
  /A/cmd/twist  /A/cmd/enable                                      (MulticopterVelocityControl)
"""
from __future__ import annotations

import asyncio
import datetime as dt
import logging
import threading
import time
from pathlib import Path
from typing import Any, Callable

from ... import config
from ...models import (EntityInfo, EntityState, Geometry, GpsReading, ImuReading, LinkDesc, ModelDesc,
                       Pose, Quat, SceneDesc, SimStatus, Vec3, VelocityAction, Visual)
from ...scenario import CameraSpec, Scenario
from ..base import EngineEvent, RawFrame, SimulationEngine
from . import sdf as sdfgen
from .process import GazeboProcess
from .transport import Transport

log = logging.getLogger(__name__)

LANDING_SURFACES = ("ground", "pad_")       # contact with these = landing/takeoff, not collision
CONTACT_COOLDOWN_S = 0.5                    # sim seconds before the same pair re-emits a collision
GROUND_CONTACT_TIMEOUT_S = 0.35             # sim seconds without ground contact -> airborne
SAFE_REMOVAL_Z = 500.0                      # bodies are lifted here before deletion (see remove())


def _t(msg_time) -> float:
    return msg_time.sec + msg_time.nsec * 1e-9


def _pose_from_msg(p) -> Pose:
    return Pose(position=Vec3(x=p.position.x, y=p.position.y, z=p.position.z),
                orientation=Quat(x=p.orientation.x, y=p.orientation.y, z=p.orientation.z, w=p.orientation.w))


def _rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate body-frame vector v into the world frame by quaternion q."""
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v.x, v.y, v.z
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return Vec3(x=vx + w * tx + (y * tz - z * ty),
                y=vy + w * ty + (z * tx - x * tz),
                z=vz + w * tz + (x * ty - y * tx))


class _RateMeter:
    def __init__(self) -> None:
        self.count = 0
        self.t0 = time.monotonic()
        self.rate = 0.0

    def tick(self) -> None:
        self.count += 1
        now = time.monotonic()
        if now - self.t0 >= 1.0:
            self.rate = self.count / (now - self.t0)
            self.count, self.t0 = 0, now


class _AgentIO:
    def __init__(self, agent_id: str, camera: CameraSpec | None, mounts=None) -> None:
        self.id = agent_id
        self.camera = camera
        self.mounts = list(mounts or [])          # image-producing SensorMounts
        self.odom = None
        self.odom_time = 0.0
        self.prev_vel: Vec3 | None = None
        self.prev_vel_time = 0.0
        self.accel: Vec3 | None = None
        self.imu = None
        self.navsat = None
        self.frames: dict[str, RawFrame] = {}
        self.rates: dict[str, _RateMeter] = {}
        self.seq: dict[str, int] = {}
        # contact bookkeeping
        self.last_ground_contact: float = -1.0
        self.grounded: bool | None = None
        self.pair_last: dict[str, float] = {}

    def reset_transients(self) -> None:
        self.odom = self.imu = self.navsat = None
        self.prev_vel = self.accel = None
        self.frames.clear()
        self.last_ground_contact = -1.0
        self.grounded = None
        self.pair_last.clear()


class GazeboEngine(SimulationEngine):
    def __init__(self) -> None:
        self.proc = GazeboProcess()
        self.tp: Transport | None = None
        self.scenario: Scenario | None = None
        self.world = ""
        self.run_dir: Path | None = None
        self.rendering = False
        self._seed = 0

        self._lock = threading.RLock()
        self._stats = None
        self._scene_msg = None
        self._model_ids: dict[int, str] = {}
        self._poses: dict[str, Pose] = {}
        self._agents: dict[str, _AgentIO] = {}
        self._kinds: dict[str, str] = {}
        self._dynamic: set[str] = set()
        self._pose_cb: Callable[[float, dict[str, Pose]], None] | None = None
        self._event_cb: Callable[[EngineEvent], None] | None = None
        self._stats_event = threading.Event()
        self._want_paused = True
        self._last_pose_time = 0.0
        self._derived_iterations = 0
        self._stats_wall = 0.0            # arrival time of the last stats message
        self._reset_wall = 0.0            # time of the last observed rewind; older stats are stale

    # ------------------------------------------------------------------ lifecycle
    @property
    def seed(self) -> int:
        return self._seed

    async def start(self, scenario: Scenario, *, seed: int | None = None) -> None:
        self.scenario = scenario
        self.world = scenario.world.name
        self.rendering = scenario.rendering
        self._seed = scenario.simulation.seed if seed is None else int(seed)

        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.run_dir = config.RUNS_DIR / f"{stamp}_{scenario.name}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        scenario.save(self.run_dir / "scenario.yaml")

        base = (config.WORLDS_DIR / scenario.world.file).read_text()
        world_file = self.run_dir / "world.sdf"
        world_file.write_text(sdfgen.build_world_sdf(base, scenario))

        start_paused = scenario.simulation.start_paused or scenario.simulation.mode == "stepped"
        self._want_paused = start_paused
        self.proc.start(world_file, run=not start_paused, seed=self._seed,
                        rendering=self.rendering, log_dir=self.run_dir)

        self.tp = Transport()
        g = self.tp.g
        self._stats_event = threading.Event()
        self.tp.subscribe(g["WorldStatistics"], f"/world/{self.world}/stats", self._on_stats)
        self.tp.subscribe(g["Pose_V"], f"/world/{self.world}/dynamic_pose/info", self._on_dynamic_pose)
        await asyncio.get_running_loop().run_in_executor(None, self._wait_for_world, 60.0)

        with self._lock:
            self._kinds = {a.id: a.type for a in scenario.agents}
            self._kinds.update({e.id: e.type for e in scenario.entities})
        await self._refresh_scene()
        for a in scenario.agents:
            self._attach_agent(a.id, a.camera, a.camera_mounts)
        log.info("world '%s' ready (seed=%s, rendering=%s), agents=%s",
                 self.world, self._seed, self.rendering, list(self._agents))

    def _wait_for_world(self, timeout: float) -> None:
        """Ready when the *new* server publishes stats and serves /control (a stale
        discovery entry from a previous server would make service_list() lie)."""
        deadline = time.time() + timeout
        target = f"/world/{self.world}/control"
        while time.time() < deadline:
            if not self.proc.alive():
                raise RuntimeError("gz sim exited early:\n" + self.proc.tail_log())
            if self._stats_event.wait(0.25) and target in self.tp.service_list():
                return
        raise TimeoutError(f"world '{self.world}' never became ready:\n" + self.proc.tail_log())

    @property
    def crash_log(self) -> str:
        return self.proc.tail_log(12)

    def _request_retry(self, service: str, req, rep_cls, *, attempts: int = 5, timeout_ms: int = 2000):
        last: Exception | None = None
        for _ in range(attempts):
            if not self.proc.alive():
                raise RuntimeError("gz sim is not running (crashed?); load a scenario again\n" + self.proc.tail_log(6))
            try:
                return self.tp.request(service, req, rep_cls, timeout_ms)
            except TimeoutError as e:
                last = e
                time.sleep(0.3)
        raise last  # type: ignore[misc]

    async def shutdown(self) -> None:
        if self.tp:
            self.tp.close()
            self.tp = None
        self.proc.stop()
        self._stats_event.clear()
        with self._lock:
            self._stats = None
            self._scene_msg = None
            self._model_ids.clear()
            self._poses.clear()
            self._agents.clear()
            self._kinds.clear()
            self._dynamic.clear()
        self.scenario = None

    def _control(self, **fields) -> None:
        g = self.tp.g
        msg = g["WorldControl"]()
        if "pause" in fields:
            self._want_paused = bool(fields.pop("pause"))
        msg.pause = self._want_paused            # every WorldControl re-applies pause
        if "multi_step" in fields:
            msg.multi_step = fields.pop("multi_step")
        if fields.pop("reset_all", False):
            msg.reset.all = True
        # After a reset with rendering sensors the server can stall for several seconds
        # while ogre2 re-initialises; keep the control timeout generous.
        self._request_retry(f"/world/{self.world}/control", msg, g["Boolean"], attempts=3, timeout_ms=8000)

    async def reset(self) -> None:
        """Rewind the world. Blocks until the rewind is *observed* in world stats;
        retries once and raises if Gazebo never applies it (a silent no-op would make
        an episode look reproducible while starting from the previous state)."""
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._reset_blocking)
        with self._lock:
            for a in self._agents.values():
                a.reset_transients()
            self._last_pose_time = 0.0

    def _reset_blocking(self) -> None:
        with self._lock:
            pre_iter = self._stats.iterations if self._stats else 0
            pre_t = max(_t(self._stats.sim_time) if self._stats else 0.0, self._last_pose_time)
            pre_iter = max(pre_iter, int(round(pre_t / (self.scenario.simulation.step_size if self.scenario else 0.004))))
        if pre_iter == 0:
            return                      # nothing has advanced yet; state is already initial
        for attempt in range(2):
            self._control(reset_all=True)
            deadline = time.time() + 3.0
            while time.time() < deadline:
                with self._lock:
                    it = self._stats.iterations if self._stats else pre_iter
                    pose_t = self._last_pose_time
                if it < pre_iter or (pose_t >= 0 and pose_t < pre_t - 1e-6):
                    with self._lock:
                        self._last_pose_time = 0.0    # rewound: restart the pose clock
                        self._reset_wall = time.monotonic()   # stats older than this are stale
                    return
                time.sleep(0.005)
            log.warning("reset not observed (iterations still %s), retrying", it)
        raise RuntimeError("Gazebo did not apply the world reset")

    async def pause(self) -> None:
        await asyncio.get_running_loop().run_in_executor(None, lambda: self._set_paused(True))

    async def resume(self) -> None:
        await asyncio.get_running_loop().run_in_executor(None, lambda: self._set_paused(False))

    def _set_paused(self, paused: bool) -> None:
        self._control(pause=paused)
        deadline = time.time() + 0.6
        while time.time() < deadline:
            with self._lock:
                if self._stats is not None and self._stats.paused == paused:
                    return
            time.sleep(0.01)

    async def step(self, steps: int = 1) -> None:
        steps = max(1, int(steps))
        await asyncio.get_running_loop().run_in_executor(None, self._step_blocking, steps)

    def _step_blocking(self, steps: int) -> None:
        """Advance exactly `steps` iterations. Completion is detected from the pose stream,
        which Gazebo publishes every iteration with an exact sim-time stamp (world stats
        arrive at only 10 Hz and would cap short RL steps at ~10/s)."""
        dt = self.scenario.simulation.step_size if self.scenario else 0.004
        with self._lock:
            start_t = self.sim_time()
        time.sleep(0.003)  # let in-flight action messages land in the controller systems
        self._control(pause=True, multi_step=steps)
        target_t = start_t + steps * dt
        deadline = time.time() + 5.0 + steps * dt * 4
        while time.time() < deadline:
            with self._lock:
                if self._last_pose_time >= target_t - dt * 0.5:
                    self._derived_iterations = int(round(self._last_pose_time / dt))
                    return
            time.sleep(0.001)
        log.warning("step(%d) did not complete in time (pose t=%.4f, target %.4f)", steps, self._last_pose_time, target_t)

    def status(self) -> SimStatus:
        with self._lock:
            s = self._stats
            running = self.proc.alive()
            if s is None:
                return SimStatus(running=running, world=self.world or None)
            sim_t = _t(s.sim_time) if self._stats_fresh() else 0.0
            iters = s.iterations if self._stats_fresh() else 0
            if self._last_pose_time > sim_t + 1e-9:          # pose clock is ahead of stats
                dt = self.scenario.simulation.step_size if self.scenario else 0.004
                sim_t = self._last_pose_time
                iters = int(round(sim_t / dt))
            paused = s.paused or (self._want_paused and self._last_pose_time > _t(s.sim_time) + 1e-9)
            return SimStatus(running=running, paused=paused, sim_time=sim_t,
                             real_time=_t(s.real_time), real_time_factor=s.real_time_factor,
                             iterations=iters, world=self.world)

    def sim_time(self) -> float:
        with self._lock:
            st = _t(self._stats.sim_time) if self._stats_fresh() else 0.0
            return max(st, self._last_pose_time)

    # ------------------------------------------------------------------ callbacks (transport threads)
    def _emit(self, name: str, sim_time: float, entities: list[str], data: dict[str, Any]) -> None:
        cb = self._event_cb
        if cb:
            try:
                cb((name, sim_time, entities, data))
            except Exception:
                log.exception("event callback failed")

    def _on_stats(self, msg) -> None:
        with self._lock:
            self._stats = msg
            self._stats_wall = time.monotonic()
        self._stats_event.set()

    def _stats_fresh(self) -> bool:
        """World stats arrive at 10 Hz; right after a rewind the latest message may still describe
        the pre-reset world. Caller holds the lock."""
        return self._stats is not None and self._stats_wall > self._reset_wall

    def _on_dynamic_pose(self, msg) -> None:
        with self._lock:
            sim_time = _t(msg.header.stamp) if msg.HasField("header") else (
                _t(self._stats.sim_time) if self._stats else 0.0)
            self._last_pose_time = sim_time
            changed: dict[str, Pose] = {}
            for p in msg.pose:
                name = self._model_ids.get(p.id)
                if name is None:
                    continue
                pose = _pose_from_msg(p)
                self._poses[name] = pose
                self._dynamic.add(name)
                changed[name] = pose
            transitions = [self._update_grounded(io, sim_time) for io in self._agents.values()]
            cb = self._pose_cb
        for tr in transitions:
            if tr:
                self._emit(*tr)
        if cb and changed:
            try:
                cb(sim_time, changed)
            except Exception:
                log.exception("pose callback failed")

    def _update_grounded(self, io: _AgentIO, sim_time: float):
        """Return an event tuple on landing/takeoff transition, else None. Caller holds lock."""
        if io.last_ground_contact < 0:
            return None
        grounded = (sim_time - io.last_ground_contact) <= GROUND_CONTACT_TIMEOUT_S
        prev = io.grounded
        io.grounded = grounded
        if prev is None or prev == grounded:
            return None
        pos = self._poses.get(io.id, Pose()).position
        return ("landing" if grounded else "takeoff", sim_time, [io.id],
                {"position": pos.model_dump()})

    def _make_odom_cb(self, io: _AgentIO):
        def cb(msg):
            with self._lock:
                t = _t(msg.header.stamp) if msg.HasField("header") else time.time()
                if io.odom is not None and t > io.odom_time:
                    q = _pose_from_msg(msg.pose).orientation
                    v = _rotate(q, Vec3(x=msg.twist.linear.x, y=msg.twist.linear.y, z=msg.twist.linear.z))
                    if io.prev_vel is not None:
                        dtm = t - io.prev_vel_time
                        if dtm > 0:
                            io.accel = Vec3(x=(v.x - io.prev_vel.x) / dtm, y=(v.y - io.prev_vel.y) / dtm,
                                            z=(v.z - io.prev_vel.z) / dtm)
                    io.prev_vel, io.prev_vel_time = v, t
                io.odom, io.odom_time = msg, t
        return cb

    def _make_store_cb(self, io: _AgentIO, attr: str):
        def cb(msg):
            with self._lock:
                setattr(io, attr, msg)
        return cb

    def _make_frame_cb(self, io: _AgentIO, sensor: str):
        fmt_names = None

        def cb(msg):
            nonlocal fmt_names
            if fmt_names is None:
                fmt_names = msg.DESCRIPTOR.fields_by_name["pixel_format_type"].enum_type.values_by_number
            with self._lock:
                seq = io.seq.get(sensor, 0) + 1
                io.seq[sensor] = seq
                st = _t(msg.header.stamp) if msg.HasField("header") else self.sim_time()
                io.frames[sensor] = RawFrame(bytes(msg.data), msg.width, msg.height,
                                             fmt_names[msg.pixel_format_type].name, st, seq, time.monotonic())
                io.rates.setdefault(sensor, _RateMeter()).tick()
        return cb

    def _make_contact_cb(self, io: _AgentIO):
        def cb(msg):
            events: list[EngineEvent] = []
            with self._lock:
                # Contacts carries one stamp for the whole batch; per-contact headers are
                # usually absent and the stats clock is too coarse (10 Hz) to use here.
                t_batch = _t(msg.header.stamp) if msg.HasField("header") else self.sim_time()
                for c in msg.contact:
                    t = _t(c.header.stamp) if c.HasField("header") and c.header.stamp.sec + c.header.stamp.nsec else t_batch
                    n1, n2 = c.collision1.name, c.collision2.name
                    other_coll = n2 if n1.startswith(io.id + "::") else n1
                    other = other_coll.split("::")[0]
                    pos = None
                    if len(c.position):
                        p = c.position[0]
                        pos = {"x": p.x, "y": p.y, "z": p.z}
                    if other.startswith(LANDING_SURFACES):
                        io.last_ground_contact = t
                        continue
                    last = io.pair_last.get(other, -1e9)
                    io.pair_last[other] = t
                    if t - last > CONTACT_COOLDOWN_S:
                        depth = float(c.depth[0]) if len(c.depth) else None
                        events.append(("collision", t, [io.id, other],
                                       {"position": pos, "collision": other_coll, "depth": depth,
                                        "other_kind": self._kind(other)}))
            for e in events:
                self._emit(*e)
        return cb

    def on_pose_update(self, cb) -> None:
        self._pose_cb = cb

    def on_event(self, cb) -> None:
        self._event_cb = cb

    # ------------------------------------------------------------------ scene / entities
    async def _refresh_scene(self) -> None:
        g = self.tp.g
        loop = asyncio.get_running_loop()
        scene = await loop.run_in_executor(
            None, lambda: self._request_retry(f"/world/{self.world}/scene/info", g["Empty"](), g["Scene"]))
        with self._lock:
            self._scene_msg = scene
            self._model_ids = {m.id: m.name for m in scene.model}
            for m in scene.model:
                self._poses.setdefault(m.name, _pose_from_msg(m.pose))

    async def _wait_for_model(self, entity_id: str, *, present: bool, timeout: float = 5.0) -> None:
        """UserCommands are executed on the server's next update; poll the scene until
        the entity (dis)appears instead of guessing a delay."""
        deadline = time.time() + timeout
        while True:
            await self._refresh_scene()
            with self._lock:
                found = entity_id in self._model_ids.values()
            if found == present:
                return
            if time.time() > deadline:
                raise TimeoutError(f"entity {entity_id!r} did not {'appear' if present else 'disappear'} in time")
            await asyncio.sleep(0.1)

    def _attach_agent(self, agent_id: str, camera: CameraSpec | None, mounts=None) -> None:
        g = self.tp.g
        io = _AgentIO(agent_id, camera, mounts)
        with self._lock:
            self._agents[agent_id] = io
            self._kinds.setdefault(agent_id, "drone")
        # No wall-clock throttling: observation freshness must be a function of sim time only.
        self.tp.subscribe(g["Odometry"], f"/model/{agent_id}/odometry", self._make_odom_cb(io))
        self.tp.subscribe(g["IMU"], f"/{agent_id}/imu", self._make_store_cb(io, "imu"))
        self.tp.subscribe(g["NavSat"], f"/{agent_id}/navsat", self._make_store_cb(io, "navsat"))
        self.tp.subscribe(g["Contacts"], f"/{agent_id}/contacts", self._make_contact_cb(io))
        if self.rendering:
            for m in io.mounts:
                self.tp.subscribe(g["Image"], f"/{agent_id}/{m.name}", self._make_frame_cb(io, m.name))
        # Advertise command topics now: gz-transport drops messages published before
        # discovery connects publisher and subscriber.
        self.tp.publisher(f"/{agent_id}/cmd/twist", g["Twist"])
        self.tp.publisher(f"/{agent_id}/cmd/enable", g["Boolean"])

    def _detach_agent(self, agent_id: str) -> None:
        with self._lock:
            io = self._agents.pop(agent_id, None)
        topics = [f"/model/{agent_id}/odometry", f"/{agent_id}/imu", f"/{agent_id}/navsat", f"/{agent_id}/contacts"]
        topics += [f"/{agent_id}/{m.name}" for m in (io.mounts if io else [])]
        for t in topics:
            self.tp.unsubscribe(t)

    def _kind(self, name: str) -> str:
        k = self._kinds.get(name)
        if k:
            return k
        if name == "ground":
            return "static"
        if name not in self._dynamic:
            return "building" if any(s in name for s in ("tower", "block", "deck")) else "obstacle"
        return "dynamic"

    async def scene(self) -> SceneDesc:
        await self._refresh_scene()
        with self._lock:
            models = [self._convert_model(m) for m in self._scene_msg.model]
        return SceneDesc(world=self.world, models=models,
                         bounds=self.scenario.world.bounds if self.scenario else None)

    def _convert_model(self, m) -> ModelDesc:
        links = []
        for l in m.link:
            visuals = [Visual(name=v.name, pose=_pose_from_msg(v.pose) if v.HasField("pose") else Pose(),
                              geometry=_convert_geometry(v.geometry), color=_color(v)) for v in l.visual]
            links.append(LinkDesc(name=l.name, pose=_pose_from_msg(l.pose) if l.HasField("pose") else Pose(),
                                  visuals=visuals))
        return ModelDesc(entity_id=m.name, kind=self._kind(m.name), is_agent=m.name in self._agents,
                         pose=self._poses.get(m.name, _pose_from_msg(m.pose)), links=links,
                         is_static=m.name not in self._dynamic)

    def list_entities(self) -> list[EntityInfo]:
        with self._lock:
            return [EntityInfo(entity_id=n, kind=self._kind(n), is_agent=n in self._agents, pose=p)
                    for n, p in self._poses.items()]

    def agent_ids(self) -> list[str]:
        with self._lock:
            return list(self._agents)

    def entity_state(self, entity_id: str) -> EntityState | None:
        with self._lock:
            pose = self._poses.get(entity_id)
            if pose is None:
                return None
            sim_time = self.sim_time()
            io = self._agents.get(entity_id)
            if io and io.odom is not None:
                q = pose.orientation
                lin = _rotate(q, Vec3(x=io.odom.twist.linear.x, y=io.odom.twist.linear.y, z=io.odom.twist.linear.z))
                ang = Vec3(x=io.odom.twist.angular.x, y=io.odom.twist.angular.y, z=io.odom.twist.angular.z)
                return EntityState(entity_id=entity_id, sim_time=sim_time, pose=pose, linear_velocity=lin,
                                   angular_velocity=ang, linear_acceleration=io.accel)
            return EntityState(entity_id=entity_id, sim_time=sim_time, pose=pose)

    # ------------------------------------------------------------------ agent sensors
    def imu(self, agent_id: str) -> ImuReading | None:
        with self._lock:
            io = self._agents.get(agent_id)
            if not io or io.imu is None:
                return None
            m = io.imu
            return ImuReading(
                linear_acceleration=Vec3(x=m.linear_acceleration.x, y=m.linear_acceleration.y, z=m.linear_acceleration.z),
                angular_velocity=Vec3(x=m.angular_velocity.x, y=m.angular_velocity.y, z=m.angular_velocity.z),
                orientation=Quat(x=m.orientation.x, y=m.orientation.y, z=m.orientation.z, w=m.orientation.w)
                if m.HasField("orientation") else None)

    def gps(self, agent_id: str) -> GpsReading | None:
        with self._lock:
            io = self._agents.get(agent_id)
            if not io or io.navsat is None:
                return None
            m = io.navsat
            return GpsReading(latitude_deg=m.latitude_deg, longitude_deg=m.longitude_deg, altitude=m.altitude,
                              velocity_enu=Vec3(x=m.velocity_east, y=m.velocity_north, z=m.velocity_up))

    def body_velocity(self, agent_id: str):
        with self._lock:
            io = self._agents.get(agent_id)
            if not io or io.odom is None:
                return None
            tw = io.odom.twist
            return (Vec3(x=tw.linear.x, y=tw.linear.y, z=tw.linear.z),
                    Vec3(x=tw.angular.x, y=tw.angular.y, z=tw.angular.z))

    def grounded(self, agent_id: str) -> bool | None:
        with self._lock:
            io = self._agents.get(agent_id)
            return io.grounded if io else None

    def sensor_names(self, agent_id: str) -> list[str]:
        with self._lock:
            io = self._agents.get(agent_id)
            names = ["imu", "navsat", "contact"]
            if io and self.rendering:
                names += [m.name for m in io.mounts]
            return names

    def sensor_mounts(self, agent_id: str) -> list:
        with self._lock:
            io = self._agents.get(agent_id)
            return list(io.mounts) if io else []

    def frame(self, agent_id: str, sensor: str) -> RawFrame | None:
        with self._lock:
            io = self._agents.get(agent_id)
            return io.frames.get(sensor) if io else None

    def sensor_rate(self, agent_id: str, sensor: str) -> float:
        with self._lock:
            io = self._agents.get(agent_id)
            m = io.rates.get(sensor) if io else None
            return m.rate if m else 0.0

    # ------------------------------------------------------------------ spawn / remove / set_pose
    def _fill_pose(self, msg, pose: Pose) -> None:
        msg.position.x, msg.position.y, msg.position.z = pose.position.x, pose.position.y, pose.position.z
        q = pose.orientation
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q.x, q.y, q.z, q.w

    async def spawn(self, entity_id: str, template: str, pose: Pose, params: dict, *, camera=None,
                    drone_type: str = "standard", mounts=None) -> bool:
        g = self.tp.g
        tpl = sdfgen.TEMPLATES.get(template)
        if tpl is None:
            raise ValueError(f"unknown template {template!r}; known: {list(sdfgen.TEMPLATES)}")
        is_agent = template in sdfgen.AGENT_TEMPLATES
        mounts = list(mounts or [])
        if is_agent and (camera is not None or mounts) and not self.rendering:
            log.warning("spawn %s: cameras requested but the world has no Sensors system; ignoring", entity_id)
            camera, mounts = None, [m for m in mounts if m.type not in ("camera", "depth")]
        req = g["EntityFactory"]()
        req.sdf = tpl(entity_id, params, camera=camera, drone_type=drone_type, mounts=mounts) if is_agent else tpl(entity_id, params)
        req.name = entity_id
        req.allow_renaming = False
        self._fill_pose(req.pose, pose)
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: self._request_retry(f"/world/{self.world}/create", req, g["Boolean"]))
        await self._wait_for_model(entity_id, present=True)
        with self._lock:
            self._kinds[entity_id] = "drone" if is_agent else {"vehicle": "vehicle", "platform": "dynamic", "beacon": "dynamic",
                                                              "obstacle": "obstacle"}.get(template, "target")
        if is_agent and entity_id in self._model_ids.values():
            img_mounts = [m for m in mounts if m.type in ("camera", "depth")]
            if camera is not None and not img_mounts:
                from ...scenario import AgentSpec
                img_mounts = AgentSpec(id=entity_id, camera=camera).camera_mounts
            self._attach_agent(entity_id, camera, img_mounts)
        return is_agent

    async def remove(self, entity_id: str) -> None:
        """Remove a model.

        gz-sim 10.5 / dartsim segfaults (GetContactsFromLastStep) when a body that was in
        contact during the last physics step is deleted — including implicitly during a
        world rewind. Workaround: teleport the body well clear of everything and run two
        iterations so the contact cache no longer references it, then delete. In stepped
        mode this advances the world by 2 iterations; callers that reset afterwards do not
        care, and callers that remove mid-episode get a documented 8 ms hiccup.
        """
        g = self.tp.g
        loop = asyncio.get_running_loop()
        with self._lock:
            pose = self._poses.get(entity_id)
            paused = self._stats.paused if self._stats else True
        if pose is not None:
            lifted = pose.model_copy(update={"position": Vec3(x=pose.position.x, y=pose.position.y, z=SAFE_REMOVAL_Z)})
            await loop.run_in_executor(None, lambda: self.set_pose(entity_id, lifted))
            if paused:
                await loop.run_in_executor(None, self._step_blocking, 2)
            else:
                await asyncio.sleep(0.05)
        req = g["Entity"]()
        req.name = entity_id
        req.type = g["Entity"].MODEL
        await loop.run_in_executor(None, lambda: self._request_retry(f"/world/{self.world}/remove", req, g["Boolean"]))
        if entity_id in self._agents:
            self._detach_agent(entity_id)
        with self._lock:
            self._poses.pop(entity_id, None)
            self._kinds.pop(entity_id, None)
        await self._wait_for_model(entity_id, present=False)

    def set_pose(self, entity_id: str, pose: Pose) -> None:
        g = self.tp.g
        msg = g["Pose"]()
        msg.name = entity_id
        self._fill_pose(msg, pose)
        # non-blocking variant: enqueued, applied on the next iteration
        try:
            self.tp.request(f"/world/{self.world}/set_pose", msg, g["Boolean"], 2000)
        except TimeoutError:
            log.warning("set_pose timed out for %s", entity_id)

    # ------------------------------------------------------------------ actions
    def send_velocity(self, agent_id: str, action: VelocityAction) -> None:
        g = self.tp.g
        vx, vy, vz = action.vx, action.vy, action.vz
        if action.frame == "world":
            with self._lock:
                pose = self._poses.get(agent_id)
            if pose is not None:
                q = pose.orientation
                inv = Quat(x=-q.x, y=-q.y, z=-q.z, w=q.w)
                v = _rotate(inv, Vec3(x=vx, y=vy, z=vz))
                vx, vy, vz = v.x, v.y, v.z
        msg = g["Twist"]()
        msg.linear.x, msg.linear.y, msg.linear.z = vx, vy, vz
        msg.angular.z = action.yaw_rate
        self.tp.publish(f"/{agent_id}/cmd/twist", msg)

    def send_arm(self, agent_id: str, armed: bool) -> None:
        g = self.tp.g
        msg = g["Boolean"]()
        msg.data = armed
        self.tp.publish(f"/{agent_id}/cmd/enable", msg)


# ---------------------------------------------------------------------- helpers
def _color(v) -> list[float] | None:
    if not v.HasField("material"):
        return None
    c = v.material.diffuse if v.material.HasField("diffuse") else v.material.ambient
    return [c.r, c.g, c.b, c.a if c.a else 1.0]


def _convert_geometry(gm) -> Geometry:
    name = gm.DESCRIPTOR.fields_by_name["type"].enum_type.values_by_number[gm.type].name
    if name == "BOX":
        return Geometry(type="box", size=Vec3(x=gm.box.size.x, y=gm.box.size.y, z=gm.box.size.z))
    if name == "CYLINDER":
        return Geometry(type="cylinder", radius=gm.cylinder.radius, length=gm.cylinder.length)
    if name == "SPHERE":
        return Geometry(type="sphere", radius=gm.sphere.radius)
    if name == "PLANE":
        return Geometry(type="plane", normal=Vec3(x=gm.plane.normal.x, y=gm.plane.normal.y, z=gm.plane.normal.z),
                        size=Vec3(x=gm.plane.size.x, y=gm.plane.size.y, z=0))
    if name == "MESH":
        return Geometry(type="mesh", uri=gm.mesh.filename,
                        scale=Vec3(x=gm.mesh.scale.x or 1, y=gm.mesh.scale.y or 1, z=gm.mesh.scale.z or 1))
    return Geometry(type="unknown")
