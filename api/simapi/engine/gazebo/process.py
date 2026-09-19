"""Manage the headless `gz sim -s` server process."""
from __future__ import annotations

import logging
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

from ... import config

log = logging.getLogger(__name__)


class GazeboProcess:
    def __init__(self) -> None:
        self.proc: subprocess.Popen | None = None
        self.log_path: Path | None = None

    def start(self, world_file: Path, *, run: bool, seed: int, verbosity: int = 2,
              rendering: bool = False, log_dir: Path | None = None) -> None:
        gz_bin = shutil.which(config.GZ_BIN)
        if gz_bin is None:
            raise RuntimeError(f"'{config.GZ_BIN}' not found on PATH; install Gazebo (brew install osrf/simulation/gz-jetty)")
        cmd = [gz_bin, "sim", "-s", "-v", str(verbosity), "--seed", str(seed)]
        if run:
            cmd.append("-r")
        if rendering:
            # --headless-rendering initialises ogre2 without a window; verified to work on
            # macOS (Metal) as well as Linux (EGL). Without it, init takes ~6 s on macOS.
            cmd += ["--headless-rendering", "--render-engine-server", "ogre2"]
        cmd.append(str(world_file))

        env = os.environ.copy()
        env.setdefault("GZ_PARTITION", "envdr3d")
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
            p for p in [str(config.MODELS_DIR), env.get("GZ_SIM_RESOURCE_PATH", "")] if p)

        log_dir = log_dir or config.RUNS_DIR
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / "gz-server.log"
        logf = open(self.log_path, "ab")
        log.info("launching: %s", " ".join(cmd))
        self.proc = subprocess.Popen(cmd, env=env, stdout=logf, stderr=subprocess.STDOUT,
                                     start_new_session=True)

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self, timeout: float = 5.0) -> None:
        if self.proc is None:
            return
        if self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
                self.proc.wait(timeout)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                    self.proc.wait(2)
                except Exception:
                    pass
        self.proc = None

    def tail_log(self, n: int = 40) -> str:
        if not self.log_path or not self.log_path.exists():
            return ""
        lines = self.log_path.read_text(errors="replace").splitlines()
        return "\n".join(lines[-n:])
