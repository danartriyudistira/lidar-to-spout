"""Model background per-bin sudut (multi-hipotesis, tetangga-aware).

Didesain untuk data LiDAR 2D yang relatif jarang: tiap bin menyimpan beberapa
kandidat jarak, pencarian mencakup bin tetangga (jitter sudut antar revolusi),
dan keyakinan turun perlahan supaya permukaan statis tetap dianggap background
walau tidak terlihat di setiap revolusi.
"""
from __future__ import annotations

from math import hypot

import numpy as np


class BackgroundModel:
    def __init__(
        self,
        bins: int = 360,
        tol_m: float = 0.12,
        learn_revs: int = 4,
        max_conf: int = 20,
        rel_tol: float = 0.03,
        slots: int = 2,
        decay_every: int = 8,
    ) -> None:
        self.bins = int(bins)
        self.tol = float(tol_m)
        self.rel_tol = float(rel_tol)
        self.learn_revs = int(learn_revs)
        self.max_conf = int(max_conf)
        self.slots = max(1, int(slots))
        self.decay_every = max(1, int(decay_every))
        self.dists = np.zeros((self.bins, self.slots), dtype=np.float64)
        self.confs = np.zeros((self.bins, self.slots), dtype=np.int32)
        self._seen = np.zeros(self.bins, dtype=np.int32)
        self._revs = 0
        self.learning = True

    def reset(self) -> None:
        self.dists[:] = 0.0
        self.confs[:] = 0
        self._revs = 0

    def _bin(self, angle: float) -> int:
        return int(round(angle / 360.0 * self.bins)) % self.bins

    def _tol(self, d: float) -> float:
        return self.tol + self.rel_tol * d

    def _neighbors(self, b: int):
        return ((b - 1) % self.bins, b, (b + 1) % self.bins)

    def _update_bin(self, b: int, d: float) -> None:
        dists = self.dists[b]
        confs = self.confs[b]
        tol = self._tol(d)
        diffs = np.abs(dists - d)
        diffs = np.where(confs > 0, diffs, np.inf)
        hit = int(np.argmin(diffs))
        if diffs[hit] <= tol:
            dists[hit] = 0.85 * dists[hit] + 0.15 * d
            confs[hit] = min(self.max_conf, confs[hit] + 1)
            return
        slot = int(np.argmin(confs))
        if confs[slot] < self.learn_revs:
            dists[slot] = d
            confs[slot] = 1

    def update(self, points: list[tuple[float, float, float]]) -> None:
        if not self.learning or not points:
            return
        self._seen[:] = 0
        for ang, x, y in points:
            d = hypot(x, y)
            b = self._bin(ang)
            self._seen[b] += 1
            self._update_bin(b, d)
            # tetangga: keyakinan lebih lemah, hanya kalau belum ada kandidat dekat
            for nb in self._neighbors(b):
                if nb == b:
                    continue
                tol = self._tol(d)
                diffs = np.abs(self.dists[nb] - d)
                diffs = np.where(self.confs[nb] > 0, diffs, np.inf)
                if np.min(diffs) > tol:
                    slot = int(np.argmin(self.confs[nb]))
                    if self.confs[nb][slot] == 0:
                        self.dists[nb][slot] = d
                        self.confs[nb][slot] = 1

        self._revs += 1
        if self._revs % self.decay_every == 0:
            missing = self._seen == 0
            self.confs[missing] = np.maximum(0, self.confs[missing] - 1)

    def is_background(self, angle: float, x: float, y: float) -> bool:
        d = hypot(x, y)
        tol = self._tol(d)
        b = self._bin(angle)
        for nb in self._neighbors(b):
            confs = self.confs[nb]
            if not np.any(confs >= self.learn_revs):
                continue
            ok = (np.abs(self.dists[nb] - d) <= tol) & (confs >= self.learn_revs)
            if np.any(ok):
                return True
        return False

    def filter(self, points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        if not points:
            return points
        return [p for p in points if not self.is_background(p[0], p[1], p[2])]

    def split(self, points: list[tuple[float, float, float]]):
        """Kembalikan (foreground, background)."""
        fg: list[tuple[float, float, float]] = []
        bg: list[tuple[float, float, float]] = []
        for p in points:
            (bg if self.is_background(p[0], p[1], p[2]) else fg).append(p)
        return fg, bg

    @property
    def learned_ratio(self) -> float:
        return float(np.mean(np.any(self.confs >= self.learn_revs, axis=1)))
