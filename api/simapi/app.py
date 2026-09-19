"""HTTP + WebSocket surface of the Simulation API (Phase 2)."""
from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Query, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from .hub import TelemetryHub
from .models import (ActionEnvelope, AgentInfo, Episode, Event, Metrics, ModeRequest, Observation,
                     ResetRequest, SimStatus, SpawnRequest, StepRequest, StepResponse)
from .scenario import Scenario
from .sensors import FrameEncoder
from .service import ActionError, SimulationService

log = logging.getLogger("simapi")


def create_app(engine_factory=None) -> FastAPI:
    hub = TelemetryHub()
    if engine_factory is None:
        from .engine.gazebo.engine import GazeboEngine
        engine_factory = GazeboEngine
    service = SimulationService(engine_factory(), hub)
    encoder = FrameEncoder()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        loop = asyncio.get_running_loop()
        hub.bind_loop(loop)
        service.bind_loop(loop)
        yield
        await service.shutdown()

    app = FastAPI(title="Simulation API", version="0.2", lifespan=lifespan,
                  description="Environment interface: observations in, actions out. Gazebo stays hidden.")
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])
    app.state.service = service

    def _err(e: Exception) -> HTTPException:
        if isinstance(e, ActionError):
            return HTTPException(422, str(e))
        if isinstance(e, TimeoutError):
            return HTTPException(503, f"simulator did not respond: {e}")
        if isinstance(e, (RuntimeError, ValueError, FileNotFoundError)):
            return HTTPException(409, str(e))
        log.exception("unhandled")
        return HTTPException(500, str(e))

    # ---- status / metrics -----------------------------------------------
    @app.get("/status", response_model=SimStatus)
    def status():
        return service.status()

    @app.get("/metrics", response_model=Metrics)
    def metrics():
        return service.metrics()

    # ---- scenarios --------------------------------------------------------
    @app.get("/scenarios")
    def scenarios():
        return service.scenarios()

    @app.get("/scenarios/{name}", response_model=Scenario)
    def scenario(name: str):
        p = service.scenario_path(name)
        if not p.exists():
            raise HTTPException(404, "scenario not found")
        return Scenario.load(p)

    @app.put("/scenarios/{name}")
    def save_scenario(name: str, scenario: Scenario):
        scenario.save(service.scenario_path(name))
        return {"saved": name}

    # ---- episode lifecycle -----------------------------------------------
    @app.post("/simulation/load/{name}", response_model=Episode)
    async def load(name: str, seed: int | None = None, mode: str | None = None):
        if not service.scenario_path(name).exists():
            raise HTTPException(404, "scenario not found")
        try:
            return await service.load_scenario(name, seed=seed, mode=mode)  # type: ignore[arg-type]
        except Exception as e:
            raise _err(e)

    @app.post("/simulation/start", response_model=Episode)
    async def start(scenario: Scenario):
        try:
            return await service.start(scenario)
        except Exception as e:
            raise _err(e)

    @app.post("/episode/reset")
    async def reset(req: ResetRequest | None = None):
        req = req or ResetRequest()
        try:
            episode, obs = await service.reset(seed=req.seed, scenario=req.scenario, mode=req.mode)
        except Exception as e:
            raise _err(e)
        return {"episode": episode, "observations": obs, "status": service.status()}

    @app.post("/simulation/reset")   # legacy alias
    async def reset_legacy(req: ResetRequest | None = None):
        return await reset(req)

    @app.get("/episode", response_model=Episode)
    def episode():
        if service.episode is None:
            raise HTTPException(404, "no episode")
        service.status()
        return service.episode

    @app.post("/simulation/pause", response_model=SimStatus)
    async def pause():
        try:
            return await service.pause()
        except Exception as e:
            raise _err(e)

    @app.post("/simulation/resume", response_model=SimStatus)
    async def resume():
        try:
            return await service.resume()
        except Exception as e:
            raise _err(e)

    @app.post("/simulation/mode", response_model=SimStatus)
    async def mode(req: ModeRequest):
        try:
            return await service.set_mode(req.mode)
        except Exception as e:
            raise _err(e)

    @app.post("/simulation/step", response_model=StepResponse)
    async def step(req: StepRequest | None = None):
        req = req or StepRequest()
        try:
            return await service.step(req.steps, req.actions, req.observe)
        except Exception as e:
            raise _err(e)

    @app.post("/simulation/shutdown")
    async def shutdown():
        await service.shutdown()
        return {"ok": True}

    # ---- world / entities -------------------------------------------------
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
        try:
            await service.spawn(req.entity_id, req.template, req.pose, req.params, observation=req.observation)
        except Exception as e:
            raise _err(e)
        return {"spawned": req.entity_id}

    @app.delete("/entities/{entity_id}")
    async def remove(entity_id: str):
        await service.remove(entity_id)
        return {"removed": entity_id}

    # ---- agents -----------------------------------------------------------
    @app.get("/agents", response_model=list[AgentInfo])
    def agents():
        return [service.agent_info(a) for a in service.agent_ids()]

    @app.get("/agents/{agent_id}", response_model=AgentInfo)
    def agent(agent_id: str):
        info = service.agent_info(agent_id)
        if info is None:
            raise HTTPException(404, "agent not found")
        return info

    @app.get("/agents/{agent_id}/observation", response_model=Observation)
    def observation(agent_id: str):
        obs = service.observation(agent_id)
        if obs is None:
            raise HTTPException(404, "agent not found")
        return obs

    @app.get("/observations", response_model=dict[str, Observation])
    def observations():
        return service.observations()

    @app.post("/agents/{agent_id}/action")
    async def action(agent_id: str, env: ActionEnvelope):
        try:
            await service.send_action(agent_id, env.action)
        except ActionError as e:
            raise HTTPException(422, str(e))
        return {"ok": True}

    @app.post("/actions")
    async def actions(body: dict[str, ActionEnvelope]):
        rejected = {}
        for aid, env in body.items():
            try:
                await service.send_action(aid, env.action)
            except ActionError as e:
                rejected[aid] = str(e)
        return {"ok": not rejected, "rejected": rejected}

    @app.get("/agents/{agent_id}/sensors/{sensor}")
    def sensor(agent_id: str, sensor: str, format: str | None = Query(None, alias="format")):
        raw = service.engine.frame(agent_id, sensor)
        if raw is None:
            if agent_id not in service.agent_ids():
                raise HTTPException(404, "agent not found")
            raise HTTPException(404, f"no frame yet for {sensor!r} (available: {service.engine.sensor_names(agent_id)})")
        try:
            data, meta = encoder.encode(raw, sensor, format)
        except ValueError as e:
            raise HTTPException(400, str(e))
        headers = {f"X-Sensor-{k}": str(v) for k, v in meta.items() if k != "content_type"}
        headers["Cache-Control"] = "no-store"
        return Response(content=data, media_type=meta["content_type"], headers=headers)

    # ---- events -----------------------------------------------------------
    @app.get("/events", response_model=list[Event])
    def events(since: int = 0, limit: int = 500):
        return service.events_since(since)[-limit:]

    # ---- websockets -------------------------------------------------------
    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        await websocket.accept()
        await hub.add(websocket)
        try:
            await websocket.send_json({"type": "hello", "status": service.status().model_dump(mode="json")})
            while True:
                msg = await websocket.receive_json()
                await _handle_ws_command(service, websocket, msg)
        except WebSocketDisconnect:
            pass
        finally:
            await hub.remove(websocket)

    @app.websocket("/ws/sensors/{agent_id}/{sensor}")
    async def ws_sensor(websocket: WebSocket, agent_id: str, sensor: str,
                        format: str | None = None, fps: float = 15.0):
        """Binary frame stream: a JSON text header then the encoded frame, per new frame."""
        await websocket.accept()
        hub.sensor_clients += 1
        last_seq = -1
        period = 1.0 / max(fps, 0.5)
        try:
            while True:
                raw = service.engine.frame(agent_id, sensor)
                if raw is not None and raw.seq != last_seq:
                    last_seq = raw.seq
                    data, meta = await asyncio.get_running_loop().run_in_executor(
                        None, encoder.encode, raw, sensor, format)
                    await websocket.send_json({k: v for k, v in meta.items()})
                    await websocket.send_bytes(data)
                await asyncio.sleep(period)
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            hub.sensor_clients -= 1

    return app


def _require_running(service: SimulationService) -> None:
    if not service.status().running:
        raise HTTPException(409, "no simulation running; load a scenario first")


async def _handle_ws_command(service: SimulationService, ws: WebSocket, msg: dict) -> None:
    """Browser commands travel over the same public operations as HTTP clients."""
    kind = msg.get("type")
    try:
        result: dict = {}
        if kind == "pause":
            await service.pause()
        elif kind == "resume":
            await service.resume()
        elif kind == "mode":
            await service.set_mode(msg["mode"])
        elif kind == "step":
            resp = await service.step(int(msg.get("steps", 1)), observe=False)
            result = {"events": [e.model_dump(mode="json") for e in resp.events]}
        elif kind == "reset":
            ep, _ = await service.reset(seed=msg.get("seed"), scenario=msg.get("scenario"), mode=msg.get("mode"))
            result = {"episode": ep.model_dump(mode="json")}
        elif kind == "action":
            env = ActionEnvelope.model_validate({"action": msg["action"]})
            await service.send_action(msg["agent_id"], env.action)
        else:
            raise ValueError(f"unknown command {kind!r}")
        await ws.send_json({"type": "ack", "for": kind, "status": service.status().model_dump(mode="json"), **result})
    except Exception as e:
        await ws.send_json({"type": "error", "for": kind, "message": str(e)})


app = create_app()
