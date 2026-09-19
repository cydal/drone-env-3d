"""WebSocket fan-out for telemetry. Thread-safe publish from engine callbacks."""
from __future__ import annotations

import asyncio
import json
from typing import Any

from fastapi import WebSocket


class TelemetryHub:
    def __init__(self) -> None:
        self._clients: set[WebSocket] = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._lock = asyncio.Lock()

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    async def add(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.add(ws)

    async def remove(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.discard(ws)

    @property
    def client_count(self) -> int:
        return len(self._clients)

    def publish_threadsafe(self, message: dict[str, Any]) -> None:
        """Call from any thread."""
        if self._loop is None or not self._clients:
            return
        self._loop.call_soon_threadsafe(lambda: asyncio.ensure_future(self.publish(message)))

    async def publish(self, message: dict[str, Any]) -> None:
        if not self._clients:
            return
        data = json.dumps(message, separators=(",", ":"))
        dead: list[WebSocket] = []
        for ws in list(self._clients):
            try:
                await ws.send_text(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            await self.remove(ws)
