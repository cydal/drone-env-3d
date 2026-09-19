"""Brief 3b: generic entity system - drone types, sensor mounts, kinematic entities, inspector detail."""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.integration


def test_drone_types_fly_and_hold(sim):
    sim.reset(scenario="entity_showcase", seed=3, mode="stepped")
    agents = {a["agent_id"]: a for a in sim.agents()}
    assert agents["drone_light"]["action_space"]["limits"]["max_speed_xy"] == 8.0      # type defaults
    assert agents["drone_heavy"]["action_space"]["limits"]["max_speed_xy"] == 4.0
    acts = {a: sim.waypoint(x, y, 6.0, speed=2.0) for a, (x, y) in (("drone_std", (0, 1.5)), ("drone_light", (0, -1.5)), ("drone_heavy", (-2, 0)))}
    sim.step(actions={a: sim.arm(True) for a in acts}, steps=1)
    sim.step(actions=acts, steps=2000)
    z = {a: sim.entity_state(a)["pose"]["position"]["z"] for a in acts}
    for a, v in z.items():
        assert 5.0 < v < 7.0, (a, v)
    sim.step(steps=1000)
    for a in acts:
        assert abs(sim.entity_state(a)["pose"]["position"]["z"] - z[a]) < 0.5, a
    assert not [e for e in sim.events() if e.event == "collision"]


def test_named_sensor_mounts_produce_frames(sim):
    sim.reset(scenario="entity_showcase", seed=3, mode="stepped")
    sim.step(steps=125)
    info = sim.agent("drone_std")
    assert {f["name"] for f in info["observation_space"]["frames"]} == {"front_cam", "down_cam", "front_depth"}
    assert set(info["sensors"]) >= {"imu", "navsat", "contact", "front_cam", "down_cam", "front_depth"}
    for name, shape in (("front_cam", (120, 160, 3)), ("down_cam", (120, 160, 3)), ("front_depth", (120, 160))):
        fr = sim.frame("drone_std", name)
        assert fr is not None and fr.shape == shape, name
    refs = {f["name"]: f for f in sim.observe("drone_std").frames}
    assert refs["down_cam"]["url"].endswith("/sensors/down_cam") and refs["front_depth"]["type"] == "depth"


def test_kinematic_entities_and_detail(sim):
    sim.reset(scenario="entity_showcase", seed=3, mode="stepped")
    b0 = sim.entity_state("beacon_01")["pose"]["orientation"]["z"]; p0 = sim.entity_state("platform_01")["pose"]["position"]
    sim.step(steps=250)
    assert abs(sim.entity_state("beacon_01")["pose"]["orientation"]["z"] - b0) > 0.05
    p1 = sim.entity_state("platform_01")["pose"]["position"]
    assert p1["y"] > p0["y"] + 0.5 and p1["z"] > p0["z"] + 0.1
    d = sim.entity_detail("platform_01")
    assert d["category"] == "dynamic" and d["trajectory"].startswith("line") and d["dimensions"]["x"] == 5.0
    d = sim.entity_detail("drone_light")
    assert d["category"] == "drone" and d["physical"]["drone_type"] == "light" and d["physical"]["mass_kg"] == 0.9
    d = sim.entity_detail("tower_01")
    assert d["category"] == "building" and d["is_static"] and d["dimensions"]["z"] == 24.0
    d = sim.entity_detail("vehicle_01")
    assert d["category"] == "vehicle" and "waypoints" in d["trajectory"]


def test_runtime_spawn_typed_drone_and_trajectory_entity(sim):
    sim.reset(scenario="collision_test", seed=7, mode="stepped")
    sim.spawn("h1", drone_type="heavy", position=(3, 3, 0.2), observation="state")
    assert sim.agent("h1")["action_space"]["limits"]["max_speed_z"] == 2.0
    sim.spawn("orb", template="beacon", position=(5, 5, 3), trajectory={"type": "rotate", "yaw_rate": 1.0})
    q0 = sim.entity_state("orb")["pose"]["orientation"]["z"]
    sim.step(steps=125)
    assert abs(sim.entity_state("orb")["pose"]["orientation"]["z"] - q0) > 0.05
    assert sim.entity_detail("orb")["trajectory"] == "rotate 1.0 rad/s"
    sim.remove("orb"); sim.remove("h1")
