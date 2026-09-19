"""Official Python client for the Simulation API (Phase 2).

    from simclient import Simulation

    sim = Simulation("localhost")
    episode = sim.reset(scenario="phase2_city", seed=42, mode="stepped")
    obs = sim.observe("drone_01")
    while not episode.done:
        action = controller(obs)
        result = sim.step(agent="drone_01", action=action)
        obs = result.observations["drone_01"]
        episode = result.episode

Talks HTTP (and WebSocket for frame streams). Knows nothing about Gazebo.
"""
from __future__ import annotations

import io
import json
import time
from dataclasses import dataclass, field
from typing import Any, Iterator

import httpx

TERMINAL = ("completed", "terminated", "failed")


# ---------------------------------------------------------------------------
# typed views (thin wrappers over the JSON the server returns)
# ---------------------------------------------------------------------------

@dataclass
class Episode:
    episode_id: str
    scenario_id: str
    seed: int
    mode: str
    status: str
    sim_time: float = 0.0
    step_count: int = 0
    iterations: int = 0
    max_sim_time: float | None = None
    log_path: str | None = None

    @property
    def done(self) -> bool:
        return self.status in TERMINAL

    @classmethod
    def from_json(cls, d: dict[str, Any]) -> "Episode":
        return cls(**{k: d.get(k) for k in cls.__dataclass_fields__})  # type: ignore[arg-type]


@dataclass
class Event:
    seq: int
    event: str
    sim_time: float
    entities: list[str]
    data: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_json(cls, d: dict[str, Any]) -> "Event":
        return cls(seq=d["seq"], event=d["event"], sim_time=d["sim_time"], entities=d["entities"], data=d.get("data", {}))


class Observation(dict):
    """Observation as a dict with convenience accessors. Fields depend on the agent's profile."""

    @property
    def agent_id(self) -> str:
        return self["agent_id"]

    @property
    def sim_time(self) -> float:
        return self["sim_time"]

    @property
    def position(self) -> tuple[float, float, float] | None:
        """Ground-truth position if the profile exposes `state` (privileged)."""
        st = self.get("state")
        if not st:
            return None
        p = st["pose"]["position"]
        return p["x"], p["y"], p["z"]

    @property
    def velocity(self) -> tuple[float, float, float] | None:
        st = self.get("state")
        if not st:
            return None
        v = st["linear_velocity"]
        return v["x"], v["y"], v["z"]

    @property
    def yaw(self) -> float | None:
        st = self.get("state")
        if not st:
            return None
        import math
        q = st["pose"]["orientation"]
        return math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))

    @property
    def frames(self) -> list[dict[str, Any]]:
        return self.get("frames", [])


@dataclass
class StepResult:
    episode: Episode
    observations: dict[str, Observation]
    events: list[Event]
    rejected_actions: dict[str, str]
    status: dict[str, Any]


# ---------------------------------------------------------------------------
# client
# ---------------------------------------------------------------------------

class Simulation:
    def __init__(self, host: str = "localhost", port: int = 8000, timeout: float = 60.0) -> None:
        self.base = f"http://{host}:{port}"
        self._http = httpx.Client(base_url=self.base, timeout=timeout,
                                  limits=httpx.Limits(max_connections=4, max_keepalive_connections=4, keepalive_expiry=120))

    # ---- lifecycle -------------------------------------------------------
    def status(self) -> dict[str, Any]:
        return self._get("/status")

    def metrics(self) -> dict[str, Any]:
        return self._get("/metrics")

    def scenarios(self) -> list[str]:
        return self._get("/scenarios")

    def reset(self, *, seed: int | None = None, scenario: str | None = None, mode: str | None = None) -> Episode:
        """Start a fresh episode. Same scenario+seed reproduces the same initial state."""
        body = {k: v for k, v in {"seed": seed, "scenario": scenario, "mode": mode}.items() if v is not None}
        r = self._post("/episode/reset", body)
        self._last_obs = {k: Observation(v) for k, v in r["observations"].items()}
        return Episode.from_json(r["episode"])

    def episode(self) -> Episode:
        return Episode.from_json(self._get("/episode"))

    def pause(self) -> dict[str, Any]:
        return self._post("/simulation/pause")

    def resume(self) -> dict[str, Any]:
        return self._post("/simulation/resume")

    def set_mode(self, mode: str) -> dict[str, Any]:
        return self._post("/simulation/mode", {"mode": mode})

    def shutdown(self) -> None:
        self._post("/simulation/shutdown")

    def close(self) -> None:
        self._http.close()

    # ---- agents -----------------------------------------------------------
    def agents(self) -> list[dict[str, Any]]:
        return self._get("/agents")

    def agent(self, agent_id: str) -> dict[str, Any]:
        return self._get(f"/agents/{agent_id}")

    def observe(self, agent_id: str) -> Observation:
        return Observation(self._get(f"/agents/{agent_id}/observation"))

    def observe_all(self) -> dict[str, Observation]:
        return {k: Observation(v) for k, v in self._get("/observations").items()}

    def act(self, agent_id: str, action: dict[str, Any]) -> None:
        """Submit an action without advancing time (realtime mode, or before step())."""
        self._post(f"/agents/{agent_id}/action", {"action": action})

    def act_many(self, actions: dict[str, dict[str, Any]]) -> dict[str, str]:
        r = self._post("/actions", {k: {"action": v} for k, v in actions.items()})
        return r["rejected"]

    def step(self, *, agent: str | None = None, action: dict[str, Any] | None = None,
             actions: dict[str, dict[str, Any]] | None = None, steps: int = 1, observe: bool = True) -> StepResult:
        """Apply actions, advance `steps` physics iterations, return new observations + events."""
        acts = dict(actions or {})
        if agent is not None and action is not None:
            acts[agent] = action
        r = self._post("/simulation/step", {"steps": steps, "actions": acts or None, "observe": observe})
        return StepResult(episode=Episode.from_json(r["episode"]),
                          observations={k: Observation(v) for k, v in r["observations"].items()},
                          events=[Event.from_json(e) for e in r["events"]],
                          rejected_actions=r["rejected_actions"], status=r["status"])

    # ---- convenience action builders -------------------------------------
    @staticmethod
    def velocity(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, frame="body") -> dict[str, Any]:
        return {"type": "velocity", "vx": vx, "vy": vy, "vz": vz, "yaw_rate": yaw_rate, "frame": frame}

    @staticmethod
    def waypoint(x, y, z, *, yaw=None, speed=2.0, tolerance=0.3) -> dict[str, Any]:
        return {"type": "waypoint", "x": x, "y": y, "z": z, "yaw": yaw, "speed": speed, "tolerance": tolerance}

    @staticmethod
    def hold() -> dict[str, Any]:
        return {"type": "hold"}

    @staticmethod
    def arm(armed: bool = True) -> dict[str, Any]:
        return {"type": "arm", "armed": armed}

    # ---- events / entities / sensors -------------------------------------
    def events(self, since: int = 0) -> list[Event]:
        return [Event.from_json(e) for e in self._get("/events", params={"since": since})]

    def entities(self) -> list[dict[str, Any]]:
        return self._get("/entities")

    def entity_state(self, entity_id: str) -> dict[str, Any]:
        """Privileged ground truth (for tooling/tests, not for policies)."""
        return self._get(f"/entities/{entity_id}")

    def spawn(self, entity_id: str, *, template: str = "quadcopter", position=(0.0, 0.0, 0.0),
              observation: str | None = None, camera: dict[str, Any] | None = None, **params) -> None:
        """Spawn an entity. `camera={...}` attaches RGB(+depth) sensors (world must have rendering)."""
        x, y, z = position
        self._post("/entities", {"entity_id": entity_id, "template": template,
                                 "pose": {"position": {"x": x, "y": y, "z": z}}, "params": params,
                                 "observation": observation, "camera": camera})

    def set_pose(self, entity_id: str, position, yaw: float = 0.0) -> None:
        """Teleport an entity (world frame). Applied on the next physics iteration."""
        import math
        x, y, z = position
        self._post(f"/entities/{entity_id}/pose", {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}})

    def remove(self, entity_id: str) -> None:
        r = self._http.delete(f"/entities/{entity_id}")
        r.raise_for_status()

    def frame(self, agent_id: str, sensor: str = "camera", fmt: str | None = None):
        """Return the latest frame as a numpy array (RGB uint8 HxWx3, or depth float32 HxW in metres)."""
        import numpy as np
        params = {"format": fmt or "raw"}
        r = self._http.get(f"/agents/{agent_id}/sensors/{sensor}", params=params)
        if r.status_code == 404:
            return None
        r.raise_for_status()
        w, h = int(r.headers["x-sensor-width"]), int(r.headers["x-sensor-height"])
        enc = r.headers["x-sensor-encoding"]
        if enc == "rgb8":
            return np.frombuffer(r.content, np.uint8).reshape(h, w, 3)
        if enc == "depth32f":
            return np.frombuffer(r.content, np.float32).reshape(h, w)
        from PIL import Image
        return np.array(Image.open(io.BytesIO(r.content)))

    def stream_frames(self, agent_id: str, sensor: str = "camera", fmt: str | None = None,
                      fps: float = 15.0) -> Iterator[tuple[dict[str, Any], bytes]]:
        """Yield (meta, encoded_bytes) from the binary WebSocket stream."""
        from websockets.sync.client import connect
        url = self.base.replace("http", "ws", 1) + f"/ws/sensors/{agent_id}/{sensor}?fps={fps}" + (f"&format={fmt}" if fmt else "")
        with connect(url) as ws:
            while True:
                meta = json.loads(ws.recv())
                data = ws.recv()
                yield meta, data

    def overlay(self, data: dict[str, Any] | None) -> None:
        """Publish task/tool annotations for the browser (target, reward, status...). Simulator-agnostic."""
        if data is None:
            self._http.delete("/overlay")
        else:
            self._post("/overlay", data)

    def wait_until_ready(self, timeout: float = 60.0) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if self.status().get("running"):
                    return
            except httpx.HTTPError:
                pass
            time.sleep(0.5)
        raise TimeoutError("simulation did not become ready")

    # ---- helpers ----------------------------------------------------------
    def _get(self, path: str, params: dict | None = None):
        r = self._send("GET", path, params=params)
        self._raise(r)
        return r.json()

    def _post(self, path: str, body: dict | None = None):
        r = self._send("POST", path, json=body)
        self._raise(r)
        return r.json()

    def _send(self, method: str, path: str, **kw) -> httpx.Response:
        """Retry transient transport errors (e.g. macOS 'Can't assign requested address' when
        ephemeral ports run out under ~200 req/s for long training runs)."""
        delay = 0.2
        for attempt in range(6):
            try:
                return self._http.request(method, path, **kw)
            except (httpx.ConnectError, httpx.RemoteProtocolError, httpx.ReadError) as e:
                if attempt == 5:
                    raise
                time.sleep(delay)
                delay = min(delay * 2, 3.0)
        raise RuntimeError("unreachable")

    @staticmethod
    def _raise(r: httpx.Response) -> None:
        if r.status_code >= 400:
            try:
                detail = r.json().get("detail", r.text)
            except Exception:
                detail = r.text
            raise SimulationError(r.status_code, detail)


class SimulationError(Exception):
    def __init__(self, status: int, detail: Any) -> None:
        super().__init__(f"{status}: {detail}")
        self.status = status
        self.detail = detail
