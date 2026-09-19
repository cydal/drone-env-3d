"""One authoritative frame source, several encodings.

Raw frames come from the engine (gz.msgs.Image payloads). This module encodes
them for transport; browsers and Python clients receive exactly the same
frames, only the container differs:

  rgb   -> jpeg (default, lossy, small) | png | raw (rgb8 bytes)
  depth -> png16 (default: uint16 millimetres, lossless) | color (jpeg colormap
           for display) | raw (float32 metres)
"""
from __future__ import annotations

import io
import threading

import numpy as np
from PIL import Image

from .engine.base import RawFrame

MAX_DEPTH_M = 65.535   # uint16 mm range


class FrameEncoder:
    def __init__(self) -> None:
        self._cache: dict[tuple, tuple[int, bytes, dict]] = {}
        self._lock = threading.Lock()

    def encode(self, raw: RawFrame, sensor: str, fmt: str | None) -> tuple[bytes, dict]:
        kind = "depth" if raw.pixel_format.startswith("R_FLOAT") else "rgb"
        fmt = fmt or ("png16" if kind == "depth" else "jpeg")
        key = (id(raw), sensor, fmt)
        with self._lock:
            hit = self._cache.get(key)
            if hit and hit[0] == raw.seq:
                return hit[1], hit[2]
        data, ctype, enc = self._encode(raw, kind, fmt)
        meta = {"content_type": ctype, "width": raw.width, "height": raw.height, "encoding": enc,
                "seq": raw.seq, "sim_time": raw.sim_time, "kind": kind}
        with self._lock:
            self._cache = {k: v for k, v in self._cache.items() if k[1] != sensor or k[2] != fmt}
            self._cache[key] = (raw.seq, data, meta)
        return data, meta

    @staticmethod
    def _encode(raw: RawFrame, kind: str, fmt: str) -> tuple[bytes, str, str]:
        w, h = raw.width, raw.height
        if kind == "rgb":
            if raw.pixel_format != "RGB_INT8":
                raise ValueError(f"unsupported pixel format {raw.pixel_format}")
            if fmt == "raw":
                return raw.data, "application/octet-stream", "rgb8"
            img = Image.frombytes("RGB", (w, h), raw.data)
            buf = io.BytesIO()
            if fmt == "png":
                img.save(buf, "PNG", compress_level=3)
                return buf.getvalue(), "image/png", "rgb8"
            img.save(buf, "JPEG", quality=80)
            return buf.getvalue(), "image/jpeg", "rgb8"

        depth = np.frombuffer(raw.data, dtype=np.float32).reshape(h, w)
        if fmt == "raw":
            return raw.data, "application/octet-stream", "depth32f"
        finite = np.isfinite(depth)
        if fmt == "color":
            d = np.where(finite, depth, 0.0)
            valid = finite & (d > 0)
            far = float(np.percentile(d[valid], 98)) if valid.any() else 1.0
            norm = np.clip(d / max(far, 1e-3), 0, 1)
            # simple inferno-ish ramp: near = bright yellow, far = dark purple
            r = np.clip(1.6 - 1.6 * norm, 0, 1)
            g = np.clip(1.2 - 1.8 * norm, 0, 1) * 0.9
            b = np.clip(0.4 + 0.8 * norm, 0, 1) * (0.3 + 0.7 * norm)
            rgb = (np.stack([r, g, b], -1) * 255).astype(np.uint8)
            rgb[~valid] = 0
            buf = io.BytesIO()
            Image.fromarray(rgb, "RGB").save(buf, "JPEG", quality=80)
            return buf.getvalue(), "image/jpeg", "depth-colormap"
        # png16: millimetres, 0 = invalid
        mm = np.where(finite, np.clip(depth, 0, MAX_DEPTH_M) * 1000.0, 0.0).astype(np.uint16)
        buf = io.BytesIO()
        Image.fromarray(mm).save(buf, "PNG", compress_level=3)
        return buf.getvalue(), "image/png", "depth16-mm"
