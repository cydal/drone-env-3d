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
from simapi.engine.base import SimulationEngine
from simapi.models import (EntityInfo, EntityState, Geometry, ImuReading, LinkDesc, ModelDesc, Pose,
                           SceneDesc, SimStatus, Vec3, VelocityAction, Visual)


class FakeEngine(SimulationEngine):
    def __init__(self):
        self.paused = True; self.t = 0.0; self.iters = 0; self.running = False
        self.agents = {}; self.actions = []; self.cb = None

    async def start(self, scenario):
        self.running = True
        self.agents = {a.id: a.spawn for a in scenario.agents}

    async def shutdown(self): self.running = False
    async def reset(self): self.t = 0.0; self.iters = 0
    async def pause(self): self.paused = True
    async def resume(self): self.paused = False
    async def step(self, steps=1):
        self.iters += steps; self.t += steps * 0.004
        if self.cb: self.cb(self.t, {k: v for k, v in self.agents.items()})
    def status(self): return SimStatus(running=self.running, paused=self.paused, sim_time=self.t, iterations=self.iters, world="w")
    async def scene(self):
        return SceneDesc(world="w", models=[ModelDesc(entity_id=k, kind="drone", is_agent=True, pose=p,
                         links=[LinkDesc(name="l", visuals=[Visual(name="v", geometry=Geometry(type="box", size=Vec3(x=1, y=1, z=1)))])])
                         for k, p in self.agents.items()])
    def list_entities(self): return [EntityInfo(entity_id=k, kind="drone", is_agent=True, pose=p) for k, p in self.agents.items()]
    def entity_state(self, eid):
        return EntityState(entity_id=eid, sim_time=self.t, pose=self.agents[eid]) if eid in self.agents else None
    def imu(self, a): return ImuReading(linear_acceleration=Vec3(z=9.8), angular_velocity=Vec3())
    def sensor_names(self, a): return ["imu"]
    def sensor_frame(self, a, s): return (b"\x00\x01", {"content_type": "application/octet-stream", "width": 1})
    async def spawn(self, eid, template, pose, params): self.agents[eid] = pose
    async def remove(self, eid): self.agents.pop(eid)
    async def send_action(self, aid, action): self.actions.append((aid, action))
    def on_pose_update(self, cb): self.cb = cb


@pytest.fixture
def client():
    app = create_app(FakeEngine)
    with TestClient(app) as c:
        yield c


def test_lifecycle_and_agents(client):
    assert client.get("/status").json()["running"] is False
    names = client.get("/scenarios").json()
    assert "test_city_two_drones" in names
    st = client.post("/simulation/load/test_city_two_drones").json()
    assert st["running"] and st["scenario"] == "test_city_two_drones" and st["seed"] == 18372

    assert client.get("/agents").json() == ["drone_01", "drone_02"]
    scene = client.get("/scene").json()
    assert len(scene["models"]) == 2 and scene["models"][0]["links"][0]["visuals"][0]["geometry"]["type"] == "box"

    obs = client.get("/agents/drone_01/observation").json()
    assert obs["state"]["pose"]["position"]["y"] == 1.0 and obs["imu"]["linear_acceleration"]["z"] == 9.8

    r = client.post("/agents/drone_01/action", json={"action": {"type": "velocity", "vz": 1.0}})
    assert r.status_code == 200
    eng: FakeEngine = client.app.state.service.engine
    assert isinstance(eng.actions[-1][1], VelocityAction) and eng.actions[-1][1].vz == 1.0
    assert client.post("/agents/nope/action", json={"action": {"type": "arm"}}).status_code == 404

    assert client.post("/simulation/resume").json()["paused"] is False
    assert client.post("/simulation/step", json={"steps": 5}).json()["iterations"] == 5
    assert client.post("/simulation/reset").json()["iterations"] == 0

    client.post("/entities", json={"entity_id": "drone_03", "pose": {"position": {"x": 3, "y": 0, "z": 0}}})
    assert "drone_03" in client.get("/agents").json()
    client.delete("/entities/drone_03")
    assert "drone_03" not in client.get("/agents").json()

    frame = client.get("/agents/drone_01/sensors/imu")
    assert frame.content == b"\x00\x01" and frame.headers["x-sensor-width"] == "1"


def test_websocket_stream(client):
    client.post("/simulation/load/test_city_two_drones")
    with client.websocket_connect("/ws") as ws:
        hello = ws.receive_json()
        assert hello["type"] == "hello" and hello["status"]["running"]
        ws.send_json({"type": "step", "steps": 2})
        msgs = [ws.receive_json(), ws.receive_json()]
        types = {m["type"] for m in msgs}
        assert "ack" in types and "state" in types
        state = next(m for m in msgs if m["type"] == "state")
        assert set(state["poses"]) == {"drone_01", "drone_02"} and len(state["poses"]["drone_01"]) == 7


def test_scenario_roundtrip(tmp_path):
    from simapi.scenario import Scenario
    sc = Scenario.load(config.SCENARIOS_DIR / "test_city_two_drones.yaml")
    sc.save(tmp_path / "copy.yaml")
    assert Scenario.load(tmp_path / "copy.yaml") == sc
