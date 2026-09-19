"""Python SDK for external controllers. Talks HTTP only; knows nothing about Gazebo."""
from __future__ import annotations

import time
from typing import Any

import httpx


class SimulationClient:
    def __init__(self, host: str = "localhost", port: int = 8000, timeout: float = 30.0) -> None:
        self.base = f"http://{host}:{port}"
        self._http = httpx.Client(base_url=self.base, timeout=timeout)

    # lifecycle
    def status(self) -> dict[str, Any]:
        return self._get("/status")

    def scenarios(self) -> list[str]:
        return self._get("/scenarios")

    def load_scenario(self, name: str) -> dict[str, Any]:
        return self._post(f"/simulation/load/{name}")

    def reset(self) -> dict[str, Any]:
        return self._post("/simulation/reset")

    def pause(self) -> dict[str, Any]:
        return self._post("/simulation/pause")

    def resume(self) -> dict[str, Any]:
        return self._post("/simulation/resume")

    def step(self, steps: int = 1) -> dict[str, Any]:
        return self._post("/simulation/step", {"steps": steps})

    def shutdown(self) -> None:
        self._post("/simulation/shutdown")

    # world
    def list_entities(self) -> list[dict[str, Any]]:
        return self._get("/entities")

    def get_entity_state(self, entity_id: str) -> dict[str, Any]:
        return self._get(f"/entities/{entity_id}")

    def spawn_entity(self, entity_id: str, template: str = "quadcopter", kind: str = "drone",
                     position=(0.0, 0.0, 0.0), **params) -> dict[str, Any]:
        x, y, z = position
        return self._post("/entities", {
            "entity_id": entity_id, "template": template, "kind": kind,
            "pose": {"position": {"x": x, "y": y, "z": z}}, "params": params})

    def remove_entity(self, entity_id: str) -> dict[str, Any]:
        r = self._http.delete(f"/entities/{entity_id}")
        r.raise_for_status()
        return r.json()

    # agents
    def agents(self) -> list[str]:
        return self._get("/agents")

    def get_observation(self, agent_id: str) -> dict[str, Any]:
        return self._get(f"/agents/{agent_id}/observation")

    def send_action(self, agent_id: str, action: dict[str, Any]) -> None:
        self._post(f"/agents/{agent_id}/action", {"action": action})

    def send_velocity(self, agent_id: str, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0) -> None:
        self.send_action(agent_id, {"type": "velocity", "vx": vx, "vy": vy, "vz": vz, "yaw_rate": yaw_rate})

    def arm(self, agent_id: str, armed: bool = True) -> None:
        self.send_action(agent_id, {"type": "arm", "armed": armed})

    def get_sensor_frame(self, agent_id: str, sensor: str) -> tuple[bytes, dict[str, str]]:
        r = self._http.get(f"/agents/{agent_id}/sensors/{sensor}")
        r.raise_for_status()
        meta = {k[9:].lower(): v for k, v in r.headers.items() if k.lower().startswith("x-sensor-")}
        meta["content_type"] = r.headers.get("content-type", "")
        return r.content, meta

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

    # helpers
    def _get(self, path: str):
        r = self._http.get(path)
        r.raise_for_status()
        return r.json()

    def _post(self, path: str, body: dict | None = None):
        r = self._http.post(path, json=body)
        r.raise_for_status()
        return r.json()
