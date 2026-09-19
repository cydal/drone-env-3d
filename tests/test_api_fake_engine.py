"""API contract tests against a fake engine (no Gazebo required)."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "api"))

import pytest
from fastapi.testclient import TestClient

from simapi import config
from simapi.app import create_app
from simapi.engine.base import RawFrame, SimulationEngine
from simapi.models import (EntityInfo, EntityState, Geometry, GpsReading, ImuReading, LinkDesc, ModelDesc,
                           Pose, SceneDesc, SimStatus, Vec3, VelocityAction, Visual)


class FakeEngine(SimulationEngine):
    def __init__(self):
        self.paused = True; self.t = 0.0; self.iters = 0; self.running = False; self._seed = 0
        self.agents: dict[str, Pose] = {}; self.entities: dict[str, Pose] = {}
        self.velocities: list = []; self.arms: list = []; self.pose_cb = None; self.event_cb = None
        self.starts = 0

    async def start(self, scenario, *, seed=None):
        self.starts += 1; self.running = True; self._seed = scenario.simulation.seed if seed is None else seed
        self.agents = {a.id: a.spawn for a in scenario.agents}
        self.entities = {e.id: e.spawn for e in scenario.entities}
        self.paused = True; self.t = 0.0; self.iters = 0
    async def shutdown(self): self.running = False
    async def reset(self): self.t = 0.0; self.iters = 0
    async def pause(self): self.paused = True
    async def resume(self): self.paused = False
    async def step(self, steps=1):
        self.iters += steps; self.t += steps * 0.004
        # move agent 0 up when a velocity was sent
        for aid, v in self.velocities[-3:]:
            p = self.agents[aid]; self.agents[aid] = p.model_copy(update={"position": Vec3(x=p.position.x, y=p.position.y, z=p.position.z + v.vz * steps * 0.004)})
        if self.pose_cb: self.pose_cb(self.t, dict(self.agents))
    def status(self): return SimStatus(running=self.running, paused=self.paused, sim_time=self.t, iterations=self.iters, world="w")
    @property
    def seed(self): return self._seed
    async def scene(self):
        return SceneDesc(world="w", models=[ModelDesc(entity_id=k, kind="drone", is_agent=True, pose=p,
                         links=[LinkDesc(name="l", visuals=[Visual(name="v", geometry=Geometry(type="box", size=Vec3(x=1, y=1, z=1)))])])
                         for k, p in self.agents.items()])
    def list_entities(self):
        return [EntityInfo(entity_id=k, kind="drone", is_agent=True, pose=p) for k, p in self.agents.items()] + \
               [EntityInfo(entity_id=k, kind="target", pose=p) for k, p in self.entities.items()]
    def entity_state(self, eid):
        if eid in self.agents: return EntityState(entity_id=eid, sim_time=self.t, pose=self.agents[eid])
        if eid in self.entities: return EntityState(entity_id=eid, sim_time=self.t, pose=self.entities[eid])
        return None
    async def spawn(self, eid, template, pose, params, *, camera=None):
        if template == "quadcopter": self.agents[eid] = pose; return True
        self.entities[eid] = pose; return False
    async def remove(self, eid): self.agents.pop(eid, None); self.entities.pop(eid, None)
    def set_pose(self, eid, pose): self.entities[eid] = pose
    def agent_ids(self): return list(self.agents)
    def imu(self, a): return ImuReading(linear_acceleration=Vec3(z=9.8), angular_velocity=Vec3())
    def gps(self, a): return GpsReading(latitude_deg=47.4, longitude_deg=8.5, altitude=488.0)
    def body_velocity(self, a): return (Vec3(x=0.1), Vec3())
    def grounded(self, a): return True
    def sensor_names(self, a): return ["imu", "navsat", "contact", "camera", "depth"]
    def frame(self, a, s):
        if s == "camera": return RawFrame(bytes([200, 30, 30]) * 4, 2, 2, "RGB_INT8", self.t, 7, 0.0)
        if s == "depth":
            import numpy as np; return RawFrame(np.array([1.0, 2.0, float("inf"), 0.5], np.float32).tobytes(), 2, 2, "R_FLOAT32", self.t, 3, 0.0)
        return None
    def sensor_rate(self, a, s): return 15.0
    def send_velocity(self, aid, action): self.velocities.append((aid, action))
    def send_arm(self, aid, armed): self.arms.append((aid, armed))
    def on_pose_update(self, cb): self.pose_cb = cb
    def on_event(self, cb): self.event_cb = cb
    # test helper
    def fake_collision(self, aid, other):
        self.event_cb(("collision", self.t, [aid, other], {"position": {"x": 1, "y": 2, "z": 3}}))


@pytest.fixture
def client():
    app = create_app(FakeEngine)
    with TestClient(app) as c:
        yield c


def test_episode_lifecycle_and_modes(client):
    assert client.get("/status").json()["running"] is False
    assert {"phase2_city", "collision_test"} <= set(client.get("/scenarios").json())
    ep = client.post("/simulation/load/phase2_city", params={"mode": "stepped", "seed": 42}).json()
    assert ep["scenario_id"] == "phase2_city" and ep["seed"] == 42 and ep["mode"] == "stepped"
    st = client.get("/status").json()
    assert st["running"] and st["mode"] == "stepped" and st["episode"]["episode_id"] == ep["episode_id"]
    # stepped mode refuses free-running
    assert client.post("/simulation/resume").status_code == 409
    # step with actions returns observations for all agents and advances exactly N iterations
    r = client.post("/simulation/step", json={"steps": 5, "actions": {"drone_03": {"type": "velocity", "vz": 1.0}}}).json()
    assert r["status"]["iterations"] == 5 and set(r["observations"]) == {"drone_01", "drone_02", "drone_03"}
    assert r["episode"]["step_count"] == 1 and r["rejected_actions"] == {}
    # reset with new seed relaunches, returns observations
    r = client.post("/episode/reset", json={"seed": 99}).json()
    assert r["episode"]["seed"] == 99 and r["episode"]["step_count"] == 0 and "drone_01" in r["observations"]
    eng: FakeEngine = client.app.state.service.engine
    assert eng.starts == 2
    # same seed -> plain rewind, no relaunch, new episode id
    old = r["episode"]["episode_id"]
    r = client.post("/episode/reset", json={"seed": 99}).json()
    assert eng.starts == 2 and r["episode"]["episode_id"] != old
    # mode switch to realtime allows resume
    assert client.post("/simulation/mode", json={"mode": "realtime"}).json()["mode"] == "realtime"
    assert client.post("/simulation/resume").json()["paused"] is False


def test_observation_profiles(client):
    client.post("/simulation/load/phase2_city", params={"mode": "stepped"})
    agents = {a["agent_id"]: a for a in client.get("/agents").json()}
    assert agents["drone_01"]["observation_space"]["components"] == ["state"]
    assert agents["drone_02"]["observation_space"]["components"] == ["gps", "imu", "velocity"]
    assert agents["drone_01"]["action_space"]["level"] == "waypoint" and "waypoint" in agents["drone_01"]["action_space"]["types"]
    assert agents["drone_03"]["action_space"]["level"] == "velocity" and "waypoint" not in agents["drone_03"]["action_space"]["types"]
    assert agents["drone_01"]["observation_space"]["frames"][0]["name"] == "camera"

    o1 = client.get("/agents/drone_01/observation").json()
    assert o1["state"] is not None and o1["gps"] is None and o1["imu"] is None
    o2 = client.get("/agents/drone_02/observation").json()
    assert o2["state"] is None and o2["gps"]["latitude_deg"] == 47.4 and o2["imu"] is not None and o2["velocity"]["linear"]["x"] == 0.1
    o3 = client.get("/agents/drone_03/observation").json()
    assert o3["state"] is not None and isinstance(o3["nearby_agents"], list) and len(o3["nearby_agents"]) == 2
    # frames are references, not inlined
    assert o1["frames"] == []  # state profile has no camera component
    allobs = client.get("/observations").json()
    assert set(allobs) == {"drone_01", "drone_02", "drone_03"}


def test_action_validation(client):
    client.post("/simulation/load/phase2_city", params={"mode": "stepped"})
    # non-finite -> 422 from pydantic
    assert client.post("/agents/drone_01/action", json={"action": {"type": "velocity", "vx": "nan"}}).status_code == 422
    # over limits -> controlled 422 with explanation
    r = client.post("/agents/drone_01/action", json={"action": {"type": "velocity", "vx": 50}})
    assert r.status_code == 422 and "horizontal speed" in r.json()["detail"]
    # waypoint on a velocity-level agent -> 422
    r = client.post("/agents/drone_03/action", json={"action": {"type": "waypoint", "x": 1, "y": 1, "z": 5}})
    assert r.status_code == 422 and "velocity-level" in r.json()["detail"]
    # unknown agent -> 422 (controlled)
    assert client.post("/agents/nope/action", json={"action": {"type": "hold"}}).status_code == 422
    # valid waypoint on waypoint-level agent -> env controller emits a velocity
    eng: FakeEngine = client.app.state.service.engine
    n = len(eng.velocities)
    assert client.post("/agents/drone_01/action", json={"action": {"type": "waypoint", "x": 5, "y": 0, "z": 4}}).status_code == 200
    assert len(eng.velocities) == n + 1 and eng.velocities[-1][1].frame == "world"
    assert client.get("/agents/drone_01").json()["control_mode"] == "waypoint"
    # batch with one bad action
    r = client.post("/actions", json={"drone_02": {"action": {"type": "hold"}}, "drone_03": {"action": {"type": "velocity", "vz": 99}}}).json()
    assert r["ok"] is False and set(r["rejected"]) == {"drone_03"}
    # rejected actions inside step are reported, step still happens
    r = client.post("/simulation/step", json={"steps": 1, "actions": {"drone_03": {"type": "velocity", "vz": 99}}}).json()
    assert "drone_03" in r["rejected_actions"] and r["status"]["iterations"] >= 1
    assert client.post("/agents/drone_01/action", json={"action": {"type": "arm", "armed": False}}).status_code == 200
    assert eng.arms[-1] == ("drone_01", False)


def test_events_and_termination(client):
    client.post("/simulation/load/collision_test")
    eng: FakeEngine = client.app.state.service.engine
    eng.fake_collision("drone_01", "tower_01")
    evs = client.get("/events").json()
    assert evs[-1]["event"] == "collision" and evs[-1]["entities"] == ["drone_01", "tower_01"]
    seq = evs[-1]["seq"]
    assert client.get("/events", params={"since": seq}).json() == []
    # new events are attached to the step response
    eng.fake_collision("drone_01", "block_02")
    r = client.post("/simulation/step", json={"steps": 1}).json()
    assert [e["event"] for e in r["events"]] == ["collision", "collision"]  # all events since the previous step
    # spawn/remove produce events and agents
    client.post("/entities", json={"entity_id": "drone_09", "pose": {"position": {"x": 3, "y": 0, "z": 0}}, "observation": "navigation"})
    assert client.get("/agents/drone_09").json()["observation_space"]["profile"] == "navigation"
    client.post("/entities", json={"entity_id": "target_09", "template": "target", "kind": "target"})
    assert "target_09" in [e["entity_id"] for e in client.get("/entities").json()]
    client.delete("/entities/drone_09")
    names = [e["event"] for e in client.get("/events").json()]
    assert "agent_spawned" in names and "agent_removed" in names


def test_sensor_frames(client):
    client.post("/simulation/load/phase2_city", params={"mode": "stepped"})
    r = client.get("/agents/drone_01/sensors/camera")
    assert r.status_code == 200 and r.headers["content-type"] == "image/jpeg" and r.headers["x-sensor-width"] == "2"
    r = client.get("/agents/drone_01/sensors/camera", params={"format": "raw"})
    assert r.content == bytes([200, 30, 30]) * 4
    r = client.get("/agents/drone_01/sensors/depth")
    assert r.headers["content-type"] == "image/png" and r.headers["x-sensor-encoding"] == "depth16-mm"
    import io
    from PIL import Image
    import numpy as np
    arr = np.array(Image.open(io.BytesIO(r.content)))
    assert arr.dtype == np.uint16 and arr.flatten().tolist() == [1000, 2000, 0, 500]
    r = client.get("/agents/drone_01/sensors/depth", params={"format": "color"})
    assert r.headers["content-type"] == "image/jpeg"
    # binary websocket stream: header + bytes
    with client.websocket_connect("/ws/sensors/drone_01/camera?fps=50") as ws:
        meta = ws.receive_json(); data = ws.receive_bytes()
        assert meta["seq"] == 7 and data[:2] == b"\xff\xd8"


def test_websocket_commands_use_public_actions(client):
    client.post("/simulation/load/phase2_city", params={"mode": "stepped"})
    eng: FakeEngine = client.app.state.service.engine
    with client.websocket_connect("/ws") as ws:
        assert ws.receive_json()["type"] == "hello"
        ws.send_json({"type": "action", "agent_id": "drone_03", "action": {"type": "velocity", "vx": 1.0}})
        msgs = [ws.receive_json()]
        while msgs[-1]["type"] != "ack":
            msgs.append(ws.receive_json())
        assert eng.velocities[-1][0] == "drone_03"
        ws.send_json({"type": "step", "steps": 2})
        m = ws.receive_json()
        while m["type"] != "ack":
            m = ws.receive_json()
        assert m["status"]["iterations"] == 2


def test_metrics_and_scenario_roundtrip(client, tmp_path):
    client.post("/simulation/load/phase2_city", params={"mode": "stepped"})
    m = client.get("/metrics").json()
    assert m["agent_count"] == 3 and m["entity_count"] == 5 and "drone_01/camera" in m["sensor_fps"]
    from simapi.scenario import Scenario
    sc = Scenario.load(config.SCENARIOS_DIR / "phase2_city.yaml")
    sc.save(tmp_path / "copy.yaml")
    assert Scenario.load(tmp_path / "copy.yaml") == sc
