"""Environment API tests against a real headless Gazebo (brief 2 §26)."""
from __future__ import annotations

import time

import pytest

pytestmark = pytest.mark.integration


def _pos(sim, aid):
    p = sim.entity_state(aid)["pose"]["position"]
    return p["x"], p["y"], p["z"]


def test_lifecycle(sim):
    ep = sim.reset(scenario="collision_test", seed=7, mode="stepped")
    assert ep.status == "paused" and ep.seed == 7 and ep.mode == "stepped"
    st = sim.status()
    assert st["running"] and st["paused"] and st["iterations"] == 0
    r = sim.step(steps=10)
    assert r.status["iterations"] == 10 and r.episode.step_count == 1
    # stepped mode never free-runs
    time.sleep(0.5)
    assert sim.status()["iterations"] == 10
    sim.set_mode("realtime")
    sim.resume(); time.sleep(0.5)
    assert sim.status()["iterations"] > 10 and not sim.status()["paused"]
    sim.pause()
    it = sim.status()["iterations"]; time.sleep(0.3)
    assert sim.status()["iterations"] == it
    ep2 = sim.reset()
    assert ep2.episode_id != ep.episode_id and sim.status()["iterations"] < 5


def test_agent_spawn_observe_act_remove(sim):
    sim.reset(scenario="collision_test", seed=7, mode="stepped")
    sim.spawn("drone_x", position=(0, 0, 0.2), observation="navigation")
    info = sim.agent("drone_x")
    assert info["observation_space"]["profile"] == "navigation" and "waypoint" not in info["action_space"]["types"]
    sim.step(steps=50)  # let it settle so sensors publish
    obs = sim.observe("drone_x")
    assert obs.get("state") is None and obs["imu"] is not None and obs["gps"] is not None
    z0 = _pos(sim, "drone_x")[2]
    sim.step(agent="drone_x", action=sim.velocity(0, 0, 1.0), steps=250)
    assert _pos(sim, "drone_x")[2] > z0 + 0.3
    assert sim.entity_state("drone_x")["linear_velocity"]["z"] > 0.1
    sim.remove("drone_x")
    assert "drone_x" not in [a["agent_id"] for a in sim.agents()]
    names = [e.event for e in sim.events()]
    assert "agent_spawned" in names and "agent_removed" in names


def test_reproducibility_same_seed(sim):
    def run():
        ep = sim.reset(scenario="collision_test", seed=7, mode="stepped")
        it0 = sim.status()["iterations"]
        r1 = sim.step(agent="drone_01", action=sim.velocity(0.4, 0.2, 1.0), steps=1)
        r = sim.step(steps=400)
        p = r.observations["drone_01"].position
        trace = (ep.episode_id, it0, r1.status["iterations"], r.status["iterations"])
        return tuple(round(v, 7) for v in p), r.status["iterations"], trace
    a, b = run(), run()
    assert a[:2] == b[:2], f"run A {a} vs run B {b}"
    assert a[1] == 401 and a[0][2] > 0.5, a


def test_multiple_agents_independent(sim):
    sim.reset(scenario="phase2_city", seed=1, mode="stepped")
    sim.step(steps=100)                      # settle on pads
    before = {a: _pos(sim, a) for a in ("drone_01", "drone_02", "drone_03")}
    sim.step(agent="drone_03", action=sim.velocity(0, 0, 1.2), steps=300)
    after = {a: _pos(sim, a) for a in before}
    assert after["drone_03"][2] > before["drone_03"][2] + 0.5
    for a in ("drone_01", "drone_02"):
        assert abs(after[a][2] - before[a][2]) < 0.05
    # observations are per-profile and independent
    obs = sim.observe_all()
    assert obs["drone_01"]["state"] is not None and obs["drone_02"]["state"] is None
    assert {n["agent_id"] for n in obs["drone_03"]["nearby_agents"]} == {"drone_01", "drone_02"}


def test_dynamic_entities_move_on_their_own(sim):
    sim.reset(scenario="phase2_city", seed=1, mode="stepped")
    t0 = _pos(sim, "target_01"); v0 = _pos(sim, "vehicle_01")
    sim.step(steps=500)                      # 2 s
    t1 = _pos(sim, "target_01"); v1 = _pos(sim, "vehicle_01")
    assert abs(t1[0] - t0[0]) + abs(t1[1] - t0[1]) > 1.0
    assert v1[0] - v0[0] > 4.0               # 4 m/s along +x


def test_collision_event(sim):
    sim.reset(scenario="collision_test", seed=7, mode="stepped")
    sim.step(agent="drone_01", action=sim.waypoint(12, 10, 6, speed=3), steps=750)
    sim.step(agent="drone_01", action=sim.waypoint(20, 10, 6, speed=4), steps=1)
    hit = None
    for _ in range(60):
        r = sim.step(steps=50)
        hit = next((e for e in r.events if e.event == "collision"), None)
        if hit:
            break
    assert hit is not None
    assert hit.entities == ["drone_01", "tower_01"] and hit.data["other_kind"] == "building"
    assert 15.5 < hit.data["position"]["x"] < 16.5


def test_action_validation_is_controlled(sim):
    from simclient.simulation import SimulationError
    sim.reset(scenario="collision_test", seed=7, mode="stepped")
    with pytest.raises(SimulationError) as ei:
        sim.act("drone_01", sim.velocity(vx=99))
    assert ei.value.status == 422
    r = sim.step(actions={"drone_01": sim.velocity(vz=99), "nope": sim.hold()}, steps=1)
    assert set(r.rejected_actions) == {"drone_01", "nope"} and r.status["iterations"] == 1
    assert sim.status()["running"]           # server unaffected


def test_frames_available(sim):
    sim.reset(scenario="phase2_city", seed=1, mode="realtime")
    sim.resume()
    frame = None
    for _ in range(40):
        frame = sim.frame("drone_01", "camera")
        if frame is not None:
            break
        time.sleep(0.25)
    assert frame is not None and frame.shape == (240, 320, 3)
    depth = sim.frame("drone_01", "depth")
    assert depth is not None and depth.shape == (240, 320) and depth.dtype.name == "float32"
    refs = sim.observe("drone_01").frames
    assert refs == []                         # state profile: no camera in observation
    sim.pause()


def test_spawn_then_reset_does_not_crash_simulator(sim):
    """Regression: rewinding with a runtime-spawned contact-sensor model crashed gz-sim
    (dartsim GetContactsFromLastStep). Runtime entities are removed before the rewind."""
    sim.reset(scenario="collision_test", seed=7, mode="stepped")
    sim.spawn("extra", position=(3, 3, 0.2))
    sim.step(steps=50)
    ep = sim.reset(seed=7)
    assert ep.status == "paused" and sim.status()["running"]
    assert "extra" not in [a["agent_id"] for a in sim.agents()]       # not part of the initial state
    r = sim.step(steps=10)
    assert r.status["iterations"] == 10
