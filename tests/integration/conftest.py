"""Integration tests need a real Gazebo (gz on PATH). They run the API in-process
on a private port and a private gz-transport partition so they never collide
with a dev server."""
from __future__ import annotations

import os
import shutil
import sys
import threading
import time
from pathlib import Path

import pytest

os.environ["GZ_PARTITION"] = "envdr3d-test"
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "api"))
sys.path.insert(0, str(ROOT / "client"))


def _free_port() -> int:
    import socket
    with socket.socket() as sck:
        sck.bind(("127.0.0.1", 0))
        return sck.getsockname()[1]


PORT = _free_port()


def pytest_collection_modifyitems(config, items):
    if shutil.which("gz") is None:
        skip = pytest.mark.skip(reason="gz not installed")
        for item in items:
            if "integration" in str(item.fspath):
                item.add_marker(skip)


@pytest.fixture(scope="session")
def server():
    import uvicorn
    from simapi.app import create_app
    app = create_app()
    cfg = uvicorn.Config(app, host="127.0.0.1", port=PORT, log_level="warning")
    srv = uvicorn.Server(cfg)
    th = threading.Thread(target=srv.run, daemon=True)
    th.start()
    deadline = time.time() + 15
    import httpx
    while time.time() < deadline:
        try:
            if httpx.get(f"http://127.0.0.1:{PORT}/status").status_code == 200:
                break
        except httpx.HTTPError:
            time.sleep(0.2)
    yield f"http://127.0.0.1:{PORT}"
    srv.should_exit = True
    th.join(timeout=10)


@pytest.fixture(scope="session")
def sim(server):
    from simclient import Simulation
    s = Simulation("127.0.0.1", PORT)
    yield s
    try:
        s.shutdown()
    except Exception:
        pass
    s.close()
