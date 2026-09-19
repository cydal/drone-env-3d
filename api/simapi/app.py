"""HTTP + WebSocket surface of the Simulation API."""
from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from . import config
from .hub import TelemetryHub
from .models import ActionEnvelope, SpawnRequest, StepRequest
from .scenario import Scenario
from .service import SimulationService

log = logging.getLogger("simapi")


def create_app(engine_factory=None) -> FastAPI:
    hub = TelemetryHub()

    if engine_factory is None:
        from .engine.gazebo.engine import GazeboEngine
        engine_factory = GazeboEngine
    service = SimulationService(engine_factory(), hub)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        hub.bind_loop(asyncio.get_running_loop())
        yield
        await service.shutdown()

    app = FastAPI(title="Simulation API", version="0.1", lifespan=lifespan)
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])
    app.state.service = service

    # ---- lifecycle -------------------------------------------------------
    @app.get("/status")
    def status():
        return service.status()

    @app.get("/scenarios")
    def scenarios():
        return service.scenarios()

    @app.get("/scenarios/{name}")
    def scenario(name: str):
        p = service.scenario_path(name)
        if not p.exists():
            raise HTTPException(404, "scenario not found")
        return Scenario.load(p)

    @app.put("/scenarios/{name}")
    def save_scenario(name: str, scenario: Scenario):
        scenario.save(service.scenario_path(name))
        return {"saved": name}

    @app.post("/simulation/load/{name}")
    async def load(name: str):
        if not service.scenario_path(name).exists():
            raise HTTPException(404, "scenario not found")
        return await service.load_scenario(name)

    @app.post("/simulation/start")
    async def start(scenario: Scenario):
        return await service.start(scenario)

    @app.post("/simulation/reset")
    async def reset():
        return await service.reset()

    @app.post("/simulation/pause")
    async def pause():
        return await service.pause()

    @app.post("/simulation/resume")
    async def resume():
        return await service.resume()

    @app.post("/simulation/step")
    async def step(req: StepRequest | None = None):
        return await service.step(req.steps if req else 1)

    @app.post("/simulation/shutdown")
    async def shutdown():
        await service.shutdown()
        return {"ok": True}

    # ---- world / entities -----------------------------------------------
    @app.get("/scene")
    async def scene():
        _require_running(service)
        return await service.scene()

    @app.get("/entities")
    def entities():
        return service.entities()

    @app.get("/entities/{entity_id}")
    def entity(entity_id: str):
        st = service.entity_state(entity_id)
        if st is None:
            raise HTTPException(404, "entity not found")
        return st

    @app.post("/entities")
    async def spawn(req: SpawnRequest):
        _require_running(service)
        await service.spawn(req.entity_id, req.template, req.pose, req.params)
        return {"spawned": req.entity_id}

    @app.delete("/entities/{entity_id}")
    async def remove(entity_id: str):
        await service.remove(entity_id)
        return {"removed": entity_id}

    # ---- agents ----------------------------------------------------------
    @app.get("/agents")
    def agents():
        return service.agent_ids()

    @app.get("/agents/{agent_id}/observation")
    def observation(agent_id: str):
        obs = service.observation(agent_id)
        if obs is None:
            raise HTTPException(404, "agent not found")
        return obs

    @app.get("/agents/{agent_id}/sensors/{sensor}")
    def sensor(agent_id: str, sensor: str):
        frame = service.sensor_frame(agent_id, sensor)
        if frame is None:
            raise HTTPException(404, "no frame")
        data, meta = frame
        headers = {f"X-Sensor-{k}": str(v) for k, v in meta.items() if k != "content_type"}
        return Response(content=data, media_type=meta.get("content_type", "application/octet-stream"), headers=headers)

    @app.post("/agents/{agent_id}/action")
    async def action(agent_id: str, env: ActionEnvelope):
        if agent_id not in service.agent_ids():
            raise HTTPException(404, "agent not found")
        await service.send_action(agent_id, env.action)
        return {"ok": True}

    # ---- telemetry stream ----------------------------------------------
    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        await websocket.accept()
        await hub.add(websocket)
        try:
            await websocket.send_json({"type": "hello", "status": service.status().model_dump()})
            while True:
                # Clients may send commands over the socket too.
                msg = await websocket.receive_json()
                await _handle_ws_command(service, websocket, msg)
        except WebSocketDisconnect:
            pass
        finally:
            await hub.remove(websocket)

    return app


def _require_running(service: SimulationService) -> None:
    if not service.status().running:
        raise HTTPException(409, "no simulation running; load a scenario first")


async def _handle_ws_command(service: SimulationService, ws: WebSocket, msg: dict) -> None:
    kind = msg.get("type")
    try:
        if kind == "pause":
            await service.pause()
        elif kind == "resume":
            await service.resume()
        elif kind == "step":
            await service.step(int(msg.get("steps", 1)))
        elif kind == "reset":
            await service.reset()
        elif kind == "action":
            env = ActionEnvelope.model_validate({"action": msg["action"]})
            await service.send_action(msg["agent_id"], env.action)
        await ws.send_json({"type": "ack", "for": kind, "status": service.status().model_dump()})
    except Exception as e:  # report, don't kill the socket
        await ws.send_json({"type": "error", "for": kind, "message": str(e)})


app = create_app()
