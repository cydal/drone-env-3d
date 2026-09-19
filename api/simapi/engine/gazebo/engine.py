"""GazeboEngine: SimulationEngine implemented on top of a headless `gz sim -s`
server driven over gz-transport (Gazebo Sim Jetty / gz-sim 10).

Topics/services used (world name W, agent id A):
  /world/W/control              service  gz.msgs.WorldControl -> Boolean   (pause/step/reset)
  /world/W/stats                topic    gz.msgs.WorldStatistics
  /world/W/scene/info           service  Empty -> gz.msgs.Scene            (entity tree + geometry)
  /world/W/dynamic_pose/info    topic    gz.msgs.Pose_V                    (moving entities @60Hz)
  /world/W/create               service  gz.msgs.EntityFactory -> Boolean  (spawn)
  /world/W/remove               service  gz.msgs.Entity -> Boolean
  /model/A/odometry             topic    gz.msgs.Odometry                  (OdometryPublisher system)
  /A/imu                        topic    gz.msgs.IMU
  /A/cmd/twist                  topic    gz.msgs.Twist   -> MulticopterVelocityControl
  /A/cmd/enable                 topic    gz.msgs.Boolean -> MulticopterVelocityControl
"""
from __future__ import annotations

import asyncio
import datetime as dt
import logging
import math
import shutil
import threading
import time
from pathlib import Path
from typing import Callable

from ... import config
from ...models import (Action, ArmAction, EntityInfo, EntityState, Geometry, ImuReading, LinkDesc,
                       ModelDesc, Pose, Quat, SceneDesc, SimStatus, Vec3, VelocityAction, Visual)
from ...scenario import Scenario
from ..base import SimulationEngine
from . import sdf as sdfgen
from .process import GazeboProcess
from .transport import Transport

log = logging.getLogger(__name__)


def _t(msg_time) -> float:
    return msg_time.sec + msg_time.nsec * 1e-9


def _pose_from_msg(p) -> Pose:
    return Pose(position=Vec3(x=p.position.x, y=p.position.y, z=p.position.z),
                orientation=Quat(x=p.orientation.x, y=p.orientation.y, z=p.orientation.z, w=p.orientation.w))


def _rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate body-frame vector v into the world frame by quaternion q."""
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v.x, v.y, v.z
    # t = 2 * cross(q.xyz, v)
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return Vec3(x=vx + w * tx + (y * tz - z * ty),
                y=vy + w * ty + (z * tx - x * tz),
                z=vz + w * tz + (x * ty - y * tx))


class _AgentIO:
    def __init__(self, agent_id: str) -> None:
        self.id = agent_id
        self.odom = None
        self.imu = None
        self.odom_time = 0.0
        self.prev_vel: Vec3 | None = None
        self.prev_vel_time = 0.0
        self.accel: Vec3 | None = None


class GazeboEngine(SimulationEngine):
    def __init__(self) -> None:
        self.proc = GazeboProcess()
        self.tp: Transport | None = None
        self.scenario: Scenario | None = None
        self.world = ""
        self.run_dir: Path | None = None
        self.rendering = False

        self._lock = threading.RLock()
        self._stats = None
        self._scene_msg = None
        self._model_ids: dict[int, str] = {}            # gz entity id -> model name (top-level)
        self._poses: dict[str, Pose] = {}
        self._agents: dict[str, _AgentIO] = {}
        self._static: set[str] = set()
        self._dynamic: set[str] = set()                 # names seen on dynamic_pose/info
        self._pose_cb: Callable[[float, dict[str, Pose]], None] | None = None
        self._stats_event = threading.Event()
        self._want_paused = True                          # desired state; observed state flickers during multi-step

    # ------------------------------------------------------------------ lifecycle
    async def start(self, scenario: Scenario) -> None:
        self.scenario = scenario
        self.world = scenario.world.name
        self.rendering = bool(scenario.agents and any(a.sensors.get("camera") for a in scenario.agents)) \
            or bool(getattr(scenario, "rendering", False))

        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.run_dir = config.RUNS_DIR / f"{stamp}_{scenario.name}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        scenario.save(self.run_dir / "scenario.yaml")

        base = (config.WORLDS_DIR / scenario.world.file).read_text()
        world_sdf = sdfgen.build_world_sdf(base, scenario, rendering=self.rendering)
        world_file = self.run_dir / "world.sdf"
        world_file.write_text(world_sdf)

        self._want_paused = scenario.simulation.start_paused
        self.proc.start(world_file, run=not scenario.simulation.start_paused,
                        seed=scenario.simulation.seed, rendering=self.rendering, log_dir=self.run_dir)

        self.tp = Transport()
        g = self.tp.g
        self._stats_event = threading.Event()
        self.tp.subscribe(g["WorldStatistics"], f"/world/{self.world}/stats", self._on_stats)
        self.tp.subscribe(g["Pose_V"], f"/world/{self.world}/dynamic_pose/info", self._on_dynamic_pose)
        await asyncio.get_running_loop().run_in_executor(None, self._wait_for_world, 60.0)
        await self._refresh_scene()
        for a in scenario.agents:
            self._attach_agent(a.id)
        log.info("world '%s' ready, agents=%s", self.world, list(self._agents))

    def _wait_for_world(self, timeout: float) -> None:
        """Ready when the *new* server publishes world stats and serves /control.

        A stale discovery entry from a previous server in this process would make a
        pure service_list() check pass too early, so we require a live stats message.
        """
        deadline = time.time() + timeout
        target = f"/world/{self.world}/control"
        while time.time() < deadline:
            if not self.proc.alive():
                raise RuntimeError("gz sim exited early:\n" + self.proc.tail_log())
            if self._stats_event.wait(0.25) and target in self.tp.service_list():
                return
        raise TimeoutError(f"world '{self.world}' never became ready:\n" + self.proc.tail_log())

    def _request_retry(self, service: str, req, rep_cls, *, attempts: int = 5, timeout_ms: int = 2000):
        last: Exception | None = None
        for _ in range(attempts):
            try:
                return self.tp.request(service, req, rep_cls, timeout_ms)
            except TimeoutError as e:  # stale discovery entry or server still loading
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
            self._static.clear()
            self._dynamic.clear()
        self.scenario = None

    def _control(self, **fields) -> None:
        g = self.tp.g
        msg = g["WorldControl"]()
        # Every WorldControl message re-applies `pause`, so always send the *desired*
        # state explicitly (the observed state is transiently false during a multi-step).
        if "pause" in fields:
            self._want_paused = bool(fields.pop("pause"))
        msg.pause = self._want_paused
        if "multi_step" in fields:
            msg.multi_step = fields.pop("multi_step")
        if fields.pop("reset_all", False):
            msg.reset.all = True
        self._request_retry(f"/world/{self.world}/control", msg, g["Boolean"], attempts=3)

    async def reset(self) -> None:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: self._control(reset_all=True))
        with self._lock:
            for a in self._agents.values():
                a.odom = a.imu = None
                a.prev_vel = a.accel = None
        # After reset the multicopter controllers are re-created and idle until commanded.

    async def pause(self) -> None:
        await asyncio.get_running_loop().run_in_executor(None, lambda: self._set_paused(True))

    async def resume(self) -> None:
        await asyncio.get_running_loop().run_in_executor(None, lambda: self._set_paused(False))

    def _set_paused(self, paused: bool) -> None:
        self._control(pause=paused)
        # stats arrive at ~10 Hz; wait briefly so status() reflects the new state
        deadline = time.time() + 0.6
        while time.time() < deadline:
            with self._lock:
                if self._stats is not None and self._stats.paused == paused:
                    return
            time.sleep(0.01)

    async def step(self, steps: int = 1) -> None:
        """Advance exactly `steps` iterations while paused, and block until done."""
        steps = max(1, int(steps))
        await asyncio.get_running_loop().run_in_executor(None, self._step_blocking, steps)

    def _step_blocking(self, steps: int) -> None:
        with self._lock:
            start_iter = self._stats.iterations if self._stats else 0
        # Give in-flight action messages a moment to reach the controller systems before
        # the world advances (transport is asynchronous; see docs/ARCHITECTURE.md).
        time.sleep(0.005)
        self._control(pause=True, multi_step=steps)
        target = start_iter + steps
        step_size = self.scenario.simulation.step_size if self.scenario else 0.004
        deadline = time.time() + 5.0 + steps * step_size * 4  # generous: RTF may be < 1
        while time.time() < deadline:
            with self._lock:
                it = self._stats.iterations if self._stats else 0
                paused = self._stats.paused if self._stats else False
            if it >= target and paused:
                return
            time.sleep(0.01)
        log.warning("step(%d) did not complete in time (at %s/%s)", steps, it, target)

    def status(self) -> SimStatus:
        with self._lock:
            s = self._stats
            running = self.proc.alive()
            if s is None:
                return SimStatus(running=running, world=self.world or None)
            return SimStatus(running=running, paused=s.paused, sim_time=_t(s.sim_time),
                             real_time=_t(s.real_time), real_time_factor=s.real_time_factor,
                             iterations=s.iterations, world=self.world)

    # ------------------------------------------------------------------ callbacks (transport threads)
    def _on_stats(self, msg) -> None:
        with self._lock:
            self._stats = msg
        self._stats_event.set()

    def _on_dynamic_pose(self, msg) -> None:
        with self._lock:
            sim_time = _t(msg.header.stamp) if msg.HasField("header") else (
                _t(self._stats.sim_time) if self._stats else 0.0)
            changed: dict[str, Pose] = {}
            for p in msg.pose:
                name = self._model_ids.get(p.id)
                if name is None:
                    continue
                pose = _pose_from_msg(p)
                self._poses[name] = pose
                self._dynamic.add(name)
                changed[name] = pose
            cb = self._pose_cb
        if cb and changed:
            try:
                cb(sim_time, changed)
            except Exception:  # never let a consumer kill the transport thread
                log.exception("pose callback failed")

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

    def _make_imu_cb(self, io: _AgentIO):
        def cb(msg):
            with self._lock:
                io.imu = msg
        return cb

    def on_pose_update(self, cb) -> None:
        self._pose_cb = cb

    # ------------------------------------------------------------------ scene / entities
    async def _refresh_scene(self) -> None:
        g = self.tp.g
        loop = asyncio.get_running_loop()
        scene = await loop.run_in_executor(
            None, lambda: self._request_retry(f"/world/{self.world}/scene/info", g["Empty"](), g["Scene"]))
        with self._lock:
            self._scene_msg = scene
            self._model_ids = {m.id: m.name for m in scene.model}
            # gz.msgs.Scene does not carry is_static; anything never published on
            # dynamic_pose/info is static (SceneBroadcaster filters static models out).
            self._static = {m.name for m in scene.model if m.is_static or m.name not in self._dynamic}
            for m in scene.model:
                self._poses[m.name] = _pose_from_msg(m.pose)

    def _attach_agent(self, agent_id: str) -> None:
        g = self.tp.g
        io = _AgentIO(agent_id)
        with self._lock:
            self._agents[agent_id] = io
        self.tp.subscribe(g["Odometry"], f"/model/{agent_id}/odometry", self._make_odom_cb(io))
        self.tp.subscribe(g["IMU"], f"/{agent_id}/imu", self._make_imu_cb(io))
        # Advertise command topics now: gz-transport drops messages published before
        # discovery has connected publisher and subscriber, so a publisher created on the
        # first action would lose that action (fatal in a step-locked control loop).
        self.tp.publisher(f"/{agent_id}/cmd/twist", g["Twist"])
        self.tp.publisher(f"/{agent_id}/cmd/enable", g["Boolean"])

    def _kind(self, name: str):
        if name in self._agents:
            return "drone"
        if name == "ground":
            return "static"
        if name in self._static and name not in self._dynamic:
            return "building" if any(k in name for k in ("tower", "block", "deck")) else "obstacle"
        return "dynamic"

    async def scene(self) -> SceneDesc:
        await self._refresh_scene()
        with self._lock:
            models = [self._convert_model(m) for m in self._scene_msg.model]
        return SceneDesc(world=self.world, models=models)

    def _convert_model(self, m) -> ModelDesc:
        links = []
        for l in m.link:
            visuals = []
            for v in l.visual:
                visuals.append(Visual(name=v.name, pose=_pose_from_msg(v.pose) if v.HasField("pose") else Pose(),
                                      geometry=_convert_geometry(v.geometry), color=_color(v)))
            links.append(LinkDesc(name=l.name, pose=_pose_from_msg(l.pose) if l.HasField("pose") else Pose(),
                                  visuals=visuals))
        return ModelDesc(entity_id=m.name, kind=self._kind(m.name), is_agent=m.name in self._agents,
                         pose=self._poses.get(m.name, _pose_from_msg(m.pose)), links=links,
                         is_static=m.name not in self._dynamic)

    def list_entities(self) -> list[EntityInfo]:
        with self._lock:
            return [EntityInfo(entity_id=n, kind=self._kind(n), is_agent=n in self._agents, pose=p)
                    for n, p in self._poses.items()]

    def entity_state(self, entity_id: str) -> EntityState | None:
        with self._lock:
            pose = self._poses.get(entity_id)
            if pose is None:
                return None
            sim_time = _t(self._stats.sim_time) if self._stats else 0.0
            io = self._agents.get(entity_id)
            if io and io.odom is not None:
                q = pose.orientation
                lin = _rotate(q, Vec3(x=io.odom.twist.linear.x, y=io.odom.twist.linear.y, z=io.odom.twist.linear.z))
                ang = Vec3(x=io.odom.twist.angular.x, y=io.odom.twist.angular.y, z=io.odom.twist.angular.z)
                return EntityState(entity_id=entity_id, sim_time=sim_time, pose=pose, linear_velocity=lin,
                                   angular_velocity=ang, linear_acceleration=io.accel)
            return EntityState(entity_id=entity_id, sim_time=sim_time, pose=pose)

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

    def sensor_names(self, agent_id: str) -> list[str]:
        names = ["imu", "navsat"]
        if self.rendering:
            names += ["camera", "depth"]
        return names

    def sensor_frame(self, agent_id: str, sensor: str):
        return None  # image sensors land in the next slice

    # ------------------------------------------------------------------ spawn / remove / actions
    async def spawn(self, entity_id: str, template: str, pose: Pose, params: dict) -> None:
        g = self.tp.g
        tpl = sdfgen.TEMPLATES.get(template)
        if tpl is None:
            raise ValueError(f"unknown template {template!r}; known: {list(sdfgen.TEMPLATES)}")
        req = g["EntityFactory"]()
        req.sdf = tpl(entity_id, params, rendering=self.rendering)
        req.name = entity_id
        req.allow_renaming = False
        req.pose.position.x, req.pose.position.y, req.pose.position.z = pose.position.x, pose.position.y, pose.position.z
        q = pose.orientation
        req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w = q.x, q.y, q.z, q.w
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: self._request_retry(f"/world/{self.world}/create", req, g["Boolean"]))
        await asyncio.sleep(0.3)  # creation is processed on the next sim iteration
        await self._refresh_scene()
        if template in sdfgen.TEMPLATES and entity_id in self._model_ids.values():
            self._attach_agent(entity_id)
        if self.scenario is not None:
            from ...scenario import AgentSpec
            self.scenario.agents.append(AgentSpec(id=entity_id, template=template, spawn=pose, params=params))

    async def remove(self, entity_id: str) -> None:
        g = self.tp.g
        req = g["Entity"]()
        req.name = entity_id
        req.type = g["Entity"].MODEL
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: self._request_retry(f"/world/{self.world}/remove", req, g["Boolean"]))
        with self._lock:
            self._agents.pop(entity_id, None)
            self._poses.pop(entity_id, None)
        self.tp.unsubscribe(f"/model/{entity_id}/odometry")
        self.tp.unsubscribe(f"/{entity_id}/imu")
        await asyncio.sleep(0.3)
        await self._refresh_scene()

    async def send_action(self, agent_id: str, action: Action) -> None:
        g = self.tp.g
        if isinstance(action, VelocityAction):
            msg = g["Twist"]()
            msg.linear.x, msg.linear.y, msg.linear.z = action.vx, action.vy, action.vz
            msg.angular.z = action.yaw_rate
            self.tp.publish(f"/{agent_id}/cmd/twist", msg)
        elif isinstance(action, ArmAction):
            msg = g["Boolean"]()
            msg.data = action.armed
            self.tp.publish(f"/{agent_id}/cmd/enable", msg)
        else:
            raise ValueError(f"unsupported action {action!r}")


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
