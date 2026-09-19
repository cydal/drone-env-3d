"""Thin wrapper over the gz-transport Python bindings (Gazebo Jetty: gz.transport / gz.msgs).

Imports are done lazily so the rest of the API can be imported (and unit
tested) on machines without Gazebo.
"""
from __future__ import annotations

import os
import threading
from typing import Any, Callable

# All processes (this API and the gz sim server) must share a partition.
os.environ.setdefault("GZ_PARTITION", "envdr3d")

_import_lock = threading.Lock()
_gz: dict[str, Any] = {}


def gz() -> dict[str, Any]:
    """Return a dict of the gz-transport Node class and message classes."""
    with _import_lock:
        if _gz:
            return _gz
        from gz.transport import Node  # Jetty (gz-transport15) — unsuffixed module
        from gz.msgs.boolean_pb2 import Boolean
        from gz.msgs.empty_pb2 import Empty
        from gz.msgs.entity_factory_pb2 import EntityFactory
        from gz.msgs.entity_pb2 import Entity
        from gz.msgs.image_pb2 import Image
        from gz.msgs.imu_pb2 import IMU
        from gz.msgs.navsat_pb2 import NavSat
        from gz.msgs.odometry_pb2 import Odometry
        from gz.msgs.pose_pb2 import Pose
        from gz.msgs.pose_v_pb2 import Pose_V
        from gz.msgs.scene_pb2 import Scene
        from gz.msgs.stringmsg_v_pb2 import StringMsg_V
        from gz.msgs.twist_pb2 import Twist
        from gz.msgs.world_control_pb2 import WorldControl
        from gz.msgs.world_stats_pb2 import WorldStatistics
        _gz.update(dict(
            Node=Node, Boolean=Boolean, Empty=Empty, EntityFactory=EntityFactory, Entity=Entity,
            Image=Image, IMU=IMU, NavSat=NavSat, Odometry=Odometry, Pose=Pose, Pose_V=Pose_V,
            Scene=Scene, StringMsg_V=StringMsg_V, Twist=Twist, WorldControl=WorldControl,
            WorldStatistics=WorldStatistics,
        ))
        return _gz


class Transport:
    def __init__(self) -> None:
        self.g = gz()
        self.node = self.g["Node"]()
        self._pubs: dict[str, Any] = {}
        self._subs: set[str] = set()

    def subscribe(self, msg_cls, topic: str, cb: Callable[[Any], None]) -> bool:
        ok = self.node.subscribe(msg_cls, topic, cb)
        if ok:
            self._subs.add(topic)
        return ok

    def unsubscribe(self, topic: str) -> None:
        if topic in self._subs:
            self.node.unsubscribe(topic)
            self._subs.discard(topic)

    def publisher(self, topic: str, msg_cls):
        pub = self._pubs.get(topic)
        if pub is None:
            pub = self.node.advertise(topic, msg_cls)
            if not pub:
                raise RuntimeError(f"advertise failed: {topic}")
            self._pubs[topic] = pub
        return pub

    def publish(self, topic: str, msg) -> bool:
        return bool(self.publisher(topic, type(msg)).publish(msg))

    def request(self, service: str, req, rep_cls, timeout_ms: int = 5000):
        ok, rep = self.node.request(service, req, type(req), rep_cls, timeout_ms)
        if not ok:
            raise TimeoutError(f"service call failed/timed out: {service}")
        return rep

    def service_list(self) -> list[str]:
        return list(self.node.service_list())

    def topic_list(self) -> list[str]:
        return list(self.node.topic_list())

    def close(self) -> None:
        for t in list(self._subs):
            self.unsubscribe(t)
        self._pubs.clear()
