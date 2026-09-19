"""Service requests in a separate process.

Why: the gz-transport Python binding's `Node.request()` blocks while holding the
GIL. gz-transport delivers subscription callbacks from its own receive thread,
which needs the GIL. With a busy set of subscriptions (poses, IMU, contacts,
cameras) that thread is almost always waiting for the GIL, so it can never
process the service *reply* — every request times out and the interpreter
freezes for the duration (measured: 15/15 timeouts at ~1.6k callbacks/s).

A worker process with its own Node and *no* subscriptions has an idle receive
thread, so its requests return in ~1 ms regardless of what the main process is
doing. Requests cross a pipe as (service, type names, bytes) and come back as
(ok, bytes). One request at a time; that is all the engine needs.
"""
from __future__ import annotations

import logging
import multiprocessing as mp
import os
import threading
from typing import Any

log = logging.getLogger(__name__)


def _worker_main(conn, partition: str) -> None:  # runs in the child process
    os.environ["GZ_PARTITION"] = partition
    from gz.transport import Node
    from google.protobuf import symbol_database
    from . import transport as _t          # loads the core gz.msgs classes
    _t.gz()
    # load *every* gz.msgs module so any service's request/response type resolves by name
    import importlib, pkgutil
    import gz.msgs as _msgs
    for m in pkgutil.iter_modules(_msgs.__path__):
        if m.name.endswith("_pb2"):
            try:
                importlib.import_module(f"gz.msgs.{m.name}")
            except Exception:
                pass
    sdb = symbol_database.Default()
    node = Node()
    while True:
        try:
            item = conn.recv()
        except (EOFError, KeyboardInterrupt):
            break
        if item is None:
            break
        op = item[0]
        try:
            if op == "request":
                _, service, req_type, req_bytes, rep_type, timeout_ms = item
                req = sdb.GetSymbol(req_type)()
                req.ParseFromString(req_bytes)
                rep_cls = sdb.GetSymbol(rep_type)
                ok, rep = node.request(service, req, type(req), rep_cls, int(timeout_ms))
                conn.send((bool(ok), rep.SerializeToString() if ok else b""))
            elif op == "service_list":
                conn.send((True, list(node.service_list())))
            elif op == "topic_list":
                conn.send((True, list(node.topic_list())))
            else:
                conn.send((False, f"unknown op {op}"))
        except Exception as e:  # report, keep serving
            conn.send((False, repr(e)))
    os._exit(0)  # skip gz-transport teardown (can segfault at interpreter exit)


class RequestWorker:
    def __init__(self, partition: str) -> None:
        ctx = mp.get_context("spawn")
        self._conn, child = ctx.Pipe()
        self._proc = ctx.Process(target=_worker_main, args=(child, partition), daemon=True, name="gz-request-worker")
        self._proc.start()
        child.close()
        self._lock = threading.Lock()

    def request(self, service: str, req, rep_cls, timeout_ms: int):
        with self._lock:
            self._conn.send(("request", service, req.DESCRIPTOR.full_name, req.SerializeToString(),
                             rep_cls.DESCRIPTOR.full_name, timeout_ms))
            if not self._conn.poll(timeout_ms / 1000 + 5.0):
                raise TimeoutError(f"request worker unresponsive for {service}")
            ok, payload = self._conn.recv()
        if not ok:
            raise TimeoutError(f"service call failed/timed out: {service}" + (f" ({payload})" if isinstance(payload, str) and payload else ""))
        rep = rep_cls()
        rep.ParseFromString(payload)
        return rep

    def _simple(self, op: str) -> Any:
        with self._lock:
            self._conn.send((op,))
            if not self._conn.poll(10.0):
                raise TimeoutError(f"request worker unresponsive for {op}")
            ok, payload = self._conn.recv()
        if not ok:
            raise RuntimeError(str(payload))
        return payload

    def service_list(self) -> list[str]:
        return self._simple("service_list")

    def topic_list(self) -> list[str]:
        return self._simple("topic_list")

    def close(self) -> None:
        try:
            with self._lock:
                self._conn.send(None)
            self._proc.join(3.0)
        except Exception:
            pass
        if self._proc.is_alive():
            self._proc.kill()
