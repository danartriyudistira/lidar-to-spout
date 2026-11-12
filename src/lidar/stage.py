"""Stage transform: ruang nyata (meter) -> kanvas, dengan posisi LiDAR bebas.

Koordinat ruang: origin kiri-atas, x ke kanan, y ke bawah, satuan meter.
LiDAR punya posisi (x, y) di dalam ruang + heading (derajat, 0 = arah +x).
"""
from __future__ import annotations

from math import cos, radians, sin

import numpy as np

from .geometry import WorldToCanvas, homography_from_quad

ASPECTS = {
    "16:9": 16.0 / 9.0,
    "32:9": 32.0 / 9.0,
    "4:3": 4.0 / 3.0,
    "1:1": 1.0,
    "9:16": 9.0 / 16.0,
}


def resolve_aspect(cfg: dict) -> float:
    st = cfg["stage"]
    room = st.get("room", {})
    name = str(st.get("canvas", {}).get("aspect", "16:9"))
    if name == "auto":
        return float(room.get("width_m", 8.0)) / max(1e-6, float(room.get("height_m", 4.0)))
    if name == "custom":
        return max(0.05, float(st["canvas"].get("aspect_custom", 2.0)))
    return ASPECTS.get(name, 16.0 / 9.0)


def compute_canvas_size(cfg: dict) -> tuple[int, int]:
    """Resolusi efektif kanvas dari pengaturan stage (auto tinggi / fixed)."""
    st = cfg.get("stage", {})
    if not st.get("enabled", True):
        return max(64, int(cfg["canvas"]["width"])), max(64, int(cfg["canvas"]["height"]))

    res = st.get("resolution", {})
    mode = str(res.get("mode", "auto_height"))
    if mode == "fixed":
        return max(64, int(cfg["canvas"]["width"])), max(64, int(cfg["canvas"]["height"]))

    aspect = resolve_aspect(cfg)
    max_w = int(res.get("max_width", 7680))
    if mode == "auto_width":
        w = max(64, int(res.get("width", 1920)))
        h = int(round(w / max(1e-6, aspect)))
    else:  # auto_height
        h = max(64, int(res.get("height", 1080)))
        w = int(round(h * aspect))
    w = min(max_w, max(64, w - w % 2))
    h = max(64, h - h % 2)
    return w, h


class _MapperBase:
    is_stage = False

    def world_to_canvas(self, xy) -> np.ndarray:  # pragma: no cover - interface
        raise NotImplementedError

    def lidar_to_room(self, xy) -> np.ndarray:  # pragma: no cover - interface
        return np.asarray(xy, dtype=np.float64).reshape(-1, 2)

    def lidar_vec_to_room(self, vx: float, vy: float) -> tuple[float, float]:  # pragma: no cover
        return (float(vx), float(vy))

    def room_to_canvas(self, xy) -> np.ndarray:
        return self.world_to_canvas(xy)

    def canvas_to_room(self, uv) -> np.ndarray:  # pragma: no cover - interface
        raise NotImplementedError

    def grid_segments(self, step: float):
        return []

    def room_outline(self):
        return []

    def lidar_marker(self):
        return None

    def px_per_m(self) -> tuple[float, float]:
        return (1.0, 1.0)


def _apply_h(H: np.ndarray, xy) -> np.ndarray:
    p = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if len(p) == 0:
        return np.zeros((0, 2))
    h = np.hstack([p, np.ones((len(p), 1))]) @ H.T
    w = h[:, 2:3]
    w[w == 0] = 1e-9
    return h[:, :2] / w


class StageMapper(_MapperBase):
    is_stage = True

    def __init__(self, cfg: dict, canvas_w: int, canvas_h: int, crop_rect=None) -> None:
        st = cfg["stage"]
        room = st.get("room", {})
        lidar = st.get("lidar", {})
        self.room_w = max(0.1, float(room.get("width_m", 8.0)))
        self.room_h = max(0.1, float(room.get("height_m", 4.0)))
        self.crop = tuple(float(v) for v in crop_rect) if crop_rect else None
        self.lx = float(lidar.get("x_m", self.room_w * 0.5))
        self.ly = float(lidar.get("y_m", self.room_h * 0.5))
        self.heading = float(lidar.get("heading_deg", 0.0))
        self.mirror = bool(lidar.get("mirror_y", False))
        self.fit = str(st.get("canvas", {}).get("fit", "stretch"))
        self.cw = max(1, int(canvas_w))
        self.ch = max(1, int(canvas_h))
        self.quad = quad_to_pixels(cfg["map"]["quad"], self.cw, self.ch,
                                   float(cfg["canvas"].get("width", self.cw)),
                                   float(cfg["canvas"].get("height", self.ch)))
        self._build()

    # ------------------------------------------------------------------
    def _build(self) -> None:
        cw, ch = float(self.cw), float(self.ch)
        if self.crop:
            x0, y0, x1, y1 = self.crop
            rw = max(1e-3, x1 - x0)
            rh = max(1e-3, y1 - y0)
        else:
            x0 = y0 = 0.0
            rw, rh = self.room_w, self.room_h
        if self.fit == "fit":
            s = min(cw / rw, ch / rh)
            sx = sy = s
            ox = (cw - rw * s) * 0.5
            oy = (ch - rh * s) * 0.5
        elif self.fit == "fill":
            s = max(cw / rw, ch / rh)
            sx = sy = s
            ox = (cw - rw * s) * 0.5
            oy = (ch - rh * s) * 0.5
        else:  # stretch
            sx, sy = cw / rw, ch / rh
            ox = oy = 0.0

        H_room = np.array([[sx, 0.0, -x0 * sx + ox],
                           [0.0, sy, -y0 * sy + oy],
                           [0.0, 0.0, 1.0]], dtype=np.float64)
        src = np.array([[0.0, 0.0], [cw, 0.0], [cw, ch], [0.0, ch]], dtype=np.float64)
        H_quad = homography_from_quad(src, self.quad)
        self.H_room = H_quad @ H_room
        self.H_room_inv = np.linalg.inv(self.H_room)

        h = radians(self.heading)
        c, s = cos(h), sin(h)
        my = -1.0 if self.mirror else 1.0
        T = np.array([[c, -s * my, self.lx], [s, c * my, self.ly], [0.0, 0.0, 1.0]], dtype=np.float64)
        self.H_lidar = self.H_room @ T
        self._sx, self._sy = sx, sy

    # ------------------------------------------------------------------
    def world_to_canvas(self, xy) -> np.ndarray:
        return _apply_h(self.H_lidar, xy)

    def lidar_to_room(self, xy) -> np.ndarray:
        p = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        if len(p) == 0:
            return np.zeros((0, 2))
        h = radians(self.heading)
        c, s = cos(h), sin(h)
        my = -1.0 if self.mirror else 1.0
        x = p[:, 0]
        y = p[:, 1] * my
        return np.column_stack([self.lx + x * c - y * s, self.ly + x * s + y * c])

    def lidar_vec_to_room(self, vx: float, vy: float) -> tuple[float, float]:
        h = radians(self.heading)
        c, s = cos(h), sin(h)
        my = -1.0 if self.mirror else 1.0
        y = vy * my
        return (vx * c - y * s, vx * s + y * c)

    def room_to_canvas(self, xy) -> np.ndarray:
        return _apply_h(self.H_room, xy)

    def canvas_to_room(self, uv) -> np.ndarray:
        return _apply_h(self.H_room_inv, uv)

    def grid_segments(self, step: float):
        step = max(0.05, float(step))
        segs = []
        nx = int(round(self.room_w / step))
        ny = int(round(self.room_h / step))
        for i in range(nx + 1):
            x = min(self.room_w, i * step)
            p = self.room_to_canvas([[x, 0.0], [x, self.room_h]])
            segs.append((p[0, 0], p[0, 1], p[1, 0], p[1, 1]))
        for j in range(ny + 1):
            y = min(self.room_h, j * step)
            p = self.room_to_canvas([[0.0, y], [self.room_w, y]])
            segs.append((p[0, 0], p[0, 1], p[1, 0], p[1, 1]))
        return segs

    def room_outline(self):
        pts = self.room_to_canvas([[0.0, 0.0], [self.room_w, 0.0],
                                   [self.room_w, self.room_h], [0.0, self.room_h], [0.0, 0.0]])
        return [tuple(p) for p in pts]

    def lidar_marker(self):
        pos = self.room_to_canvas([[self.lx, self.ly]])[0]
        h = radians(self.heading)
        fwd = self.room_to_canvas([[self.lx + cos(h), self.ly + sin(h)]])[0]
        return tuple(pos), tuple(fwd)

    def px_per_m(self) -> tuple[float, float]:
        return (float(self._sx), float(self._sy))


class ManualMapper(_MapperBase):
    """Mode lama: world_rect (relatif LiDAR) -> quad, kompatibel config v1."""

    def __init__(self, cfg: dict, canvas_w: int, canvas_h: int) -> None:
        self.cw = max(1, int(canvas_w))
        self.ch = max(1, int(canvas_h))
        quad_px = quad_to_pixels(cfg["map"]["quad"], self.cw, self.ch,
                                 float(cfg["canvas"].get("width", self.cw)),
                                 float(cfg["canvas"].get("height", self.ch)))
        self.w2c = WorldToCanvas(cfg["map"]["world_rect"], quad_px)
        self.world_rect = tuple(float(v) for v in cfg["map"]["world_rect"])
        self.rot = radians(float(cfg["map"].get("angle_offset_deg", 0.0)))
        self.mirror = bool(cfg["map"].get("mirror_y", False))

    def _rot_mirror(self, xy) -> np.ndarray:
        p = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        if len(p) == 0:
            return np.zeros((0, 2))
        if abs(self.rot) > 1e-9:
            c, s = cos(self.rot), sin(self.rot)
            p = np.column_stack([p[:, 0] * c - p[:, 1] * s, p[:, 0] * s + p[:, 1] * c])
        if self.mirror:
            p[:, 1] *= -1.0
        return p

    def world_to_canvas(self, xy) -> np.ndarray:
        return self.w2c.world_to_canvas(self._rot_mirror(xy))

    def lidar_to_room(self, xy) -> np.ndarray:
        return self._rot_mirror(xy)

    def lidar_vec_to_room(self, vx: float, vy: float) -> tuple[float, float]:
        p = self._rot_mirror([[vx, vy]])[0]
        return (float(p[0]), float(p[1]))

    def room_to_canvas(self, xy) -> np.ndarray:
        return self.w2c.world_to_canvas(np.asarray(xy, dtype=np.float64).reshape(-1, 2))

    def canvas_to_room(self, uv) -> np.ndarray:
        return self.w2c.canvas_to_world(uv)

    def grid_segments(self, step: float):
        step = max(0.05, float(step))
        x0, y0, x1, y1 = self.world_rect
        segs = []
        nx = int(round(abs(x1 - x0) / step))
        ny = int(round(abs(y1 - y0) / step))
        for i in range(nx + 1):
            x = x0 + (x1 - x0) * (i / max(1, nx))
            p = self.room_to_canvas([[x, y0], [x, y1]])
            segs.append((p[0, 0], p[0, 1], p[1, 0], p[1, 1]))
        for j in range(ny + 1):
            y = y0 + (y1 - y0) * (j / max(1, ny))
            p = self.room_to_canvas([[x0, y], [x1, y]])
            segs.append((p[0, 0], p[0, 1], p[1, 0], p[1, 1]))
        return segs

    def room_outline(self):
        x0, y0, x1, y1 = self.world_rect
        pts = self.room_to_canvas([[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]])
        return [tuple(p) for p in pts]

    def lidar_marker(self):
        pos = self.world_to_canvas([[0.0, 0.0]])[0]
        fwd = self.world_to_canvas([[cos(self.rot), sin(self.rot)]])[0]
        return tuple(pos), tuple(fwd)

    def px_per_m(self) -> tuple[float, float]:
        x0, y0, x1, y1 = self.world_rect
        p = self.room_to_canvas([[x0, y0], [x1, y1], [x0, y1]])
        w = abs(p[1][0] - p[0][0]) / max(1e-6, abs(x1 - x0))
        h = abs(p[1][1] - p[0][1]) / max(1e-6, abs(y1 - y0))
        return (float(w), float(h))


def quad_to_pixels(quad, cw: float, ch: float, ref_w: float, ref_h: float) -> np.ndarray:
    """Quad boleh dinormalisasi (0..1) atau legacy piksel; kembalikan dalam piksel kanvas."""
    q = np.asarray(quad, dtype=np.float64).reshape(4, 2)
    if float(np.min(q)) >= -0.5 and float(np.max(q)) <= 1.5:
        return q * np.array([cw, ch], dtype=np.float64)
    if ref_w <= 0 or ref_h <= 0:
        return q
    return q * np.array([cw / ref_w, ch / ref_h], dtype=np.float64)


def make_mapper(cfg: dict, canvas_w: int, canvas_h: int, crop_rect=None) -> _MapperBase:
    if cfg.get("stage", {}).get("enabled", True):
        return StageMapper(cfg, canvas_w, canvas_h, crop_rect=crop_rect)
    return ManualMapper(cfg, canvas_w, canvas_h)
