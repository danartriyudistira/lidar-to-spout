"""Geometri: polar -> kartesian, crop, homography."""
from __future__ import annotations

from math import cos, radians, sin

import numpy as np


def polar_to_xy(angle_deg: float, dist_m: float) -> tuple[float, float]:
    a = radians(angle_deg)
    return dist_m * cos(a), dist_m * sin(a)


def in_angle_range(angle: float, lo: float, hi: float) -> bool:
    """Rentang sudut yang boleh melewati 360 (mis. 300..60)."""
    span = (hi - lo) % 360.0
    if span < 1e-6:
        return True  # rentang penuh (0..360)
    angle %= 360.0
    lo %= 360.0
    delta = (angle - lo) % 360.0
    return delta <= span


def crop_points(
    points: list[tuple[float, float, float]],
    angle_min: float,
    angle_max: float,
    dist_min: float,
    dist_max: float,
    use_angle: bool = True,
) -> list[tuple[float, float, float]]:
    """points: (angle_deg, x_m, y_m)"""
    out = []
    for ang, x, y in points:
        d = (x * x + y * y) ** 0.5
        if d < dist_min or d > dist_max:
            continue
        if use_angle and not in_angle_range(ang, angle_min, angle_max):
            continue
        out.append((ang, x, y))
    return out


def homography_from_quad(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """H 3x3 yang memetakan 4 titik src -> 4 titik dst (urutan sama)."""
    A = []
    for (x, y), (u, v) in zip(src, dst):
        A.append([x, y, 1, 0, 0, 0, -u * x, -u * y])
        A.append([0, 0, 0, x, y, 1, -v * x, -v * y])
    A = np.asarray(A, dtype=np.float64)
    b = np.asarray(dst, dtype=np.float64).reshape(-1)
    h = np.linalg.solve(A, b)
    return np.append(h, 1.0).reshape(3, 3)


class WorldToCanvas:
    """Mapping world (meter) -> canvas (pixel) melalui 4 titik quad."""

    def __init__(self, world_rect, quad) -> None:
        x0, y0, x1, y1 = world_rect
        self.world_rect = (float(x0), float(y0), float(x1), float(y1))
        self.quad = np.asarray(quad, dtype=np.float64).reshape(4, 2)
        self._H = np.eye(3)
        self._Hinv = np.eye(3)
        self.rebuild()

    def rebuild(self) -> None:
        x0, y0, x1, y1 = self.world_rect
        src = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float64)
        self._H = homography_from_quad(src, self.quad)
        self._Hinv = np.linalg.inv(self._H)

    def set_quad(self, quad) -> None:
        self.quad = np.asarray(quad, dtype=np.float64).reshape(4, 2)
        self.rebuild()

    def set_world_rect(self, rect) -> None:
        self.world_rect = tuple(float(v) for v in rect)
        self.rebuild()

    def world_to_canvas(self, xy: np.ndarray) -> np.ndarray:
        """xy: (N,2) meter -> (N,2) pixel."""
        if xy is None or len(xy) == 0:
            return np.zeros((0, 2))
        p = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        h = np.hstack([p, np.ones((len(p), 1))]) @ self._H.T
        w = h[:, 2:3]
        w[w == 0] = 1e-9
        return h[:, :2] / w

    def canvas_to_world(self, uv: np.ndarray) -> np.ndarray:
        p = np.hstack([np.asarray(uv, dtype=np.float64).reshape(-1, 2), np.ones((len(uv), 1))]) @ self._Hinv.T
        w = p[:, 2:3]
        w[w == 0] = 1e-9
        return p[:, :2] / w
