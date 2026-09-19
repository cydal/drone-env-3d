"""Brief 3b: recording, replay-based snapshots/restore, replay buffer rows, world state."""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.integration


def _pos(sim, e):
    p = sim.entity_state(e)["pose"]["position"]
    return (round(p["x"], 6), round(p["y"], 6), round(p["z"], 6))


def test_snapshot_restore_is_exact_and_branchable(sim):
    sim.reset(scenario="phase2_city", seed=5, mode="stepped")
    sim.recording_start(observations=False, states=True)
    sim.step(actions={"drone_01": sim.arm(True)}, steps=1)
    for i in range(6):
        sim.step(actions={"drone_01": sim.velocity(1.0, 0.2 * (i % 2), 0.8)}, steps=50)
    snap = sim.snapshot("t")
    captured = {e: _pos(sim, e) for e in ("drone_01", "drone_02", "vehicle_01", "target_01")}
    for _ in range(4):
        sim.step(actions={"drone_01": sim.velocity(-1.0, 1.0, -0.5)}, steps=50)
    branch_a = _pos(sim, "drone_01")
    res = sim.restore(snap["snapshot_id"])
    assert res["divergence"]["max_position_m"] < 1e-6 and res["divergence"]["max_velocity_mps"] < 1e-5
    for e, p in captured.items():
        assert _pos(sim, e) == p, e
    assert sim.status()["iterations"] == snap["iteration"]
    for _ in range(4):
        sim.step(actions={"drone_01": sim.velocity(-1.0, 1.0, -0.5)}, steps=50)
    assert _pos(sim, "drone_01") == branch_a          # branching from a restored state reproduces the future


def test_recording_rows_and_replay(sim):
    sim.reset(scenario="collision_test", seed=7, mode="stepped")
    rec = sim.recording_start(observations=True, states=True)
    sim.step(actions={"drone_01": sim.arm(True)}, steps=1)
    for i in range(5):
        sim.step(actions={"drone_01": sim.velocity(0.5, 0.0, 1.0)}, steps=25)
    final = _pos(sim, "drone_01")
    rows = list(sim.iter_rows(rec["recording_id"], batch=3))
    assert len(rows) == 6 and rows[0]["iteration"] == 1 and rows[-1]["iteration"] == 126
    assert "drone_01" in rows[1]["states"] and rows[1]["observations"]["drone_01"]["state"] is not None
    assert rows[1]["actions"]["drone_01"]["type"] == "velocity"
    meta = sim.recording(rec["recording_id"])
    assert meta["actions"] and meta["actions"][0]["action"]["type"] == "arm"
    # keep flying (this is also recorded), then replay only up to iteration 126 -> the earlier state
    sim.step(actions={"drone_01": sim.velocity(-2.0, 1.0, 0.0)}, steps=100)
    perturbed = _pos(sim, "drone_01")
    rep = sim.replay(rec["recording_id"], until_iteration=126)
    assert rep["replayed_to_iteration"] == 126 and _pos(sim, "drone_01") == final
    # and a full replay reproduces the whole episode including the later actions
    rep = sim.replay(rec["recording_id"])
    assert rep["replayed_to_iteration"] == 226 and _pos(sim, "drone_01") == perturbed


def test_world_state(sim):
    sim.reset(scenario="phase2_city", seed=1, mode="stepped")
    ws = sim.world_state()
    assert ws["iteration"] == 0 and ws["mode"] == "stepped"
    ids = {e["entity_id"] for e in ws["entities"]}
    assert {"drone_01", "drone_02", "drone_03", "target_01", "vehicle_01", "tower_01"} <= ids
    assert set(ws["agents"]) == {"drone_01", "drone_02", "drone_03"} and ws["recording"]["recording_id"] == ws["episode"]["episode_id"]
