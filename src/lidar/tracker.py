"""Clustering titik scan menjadi objek (kaki) dan tracking antar revolusi."""
from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot


@dataclass(slots=True)
class Cluster:
    x: float
    y: float
    size: float
    count: int
    angle: float


@dataclass
class Track:
    id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    size: float = 0.1
    hits: int = 1
    misses: int = 0
    confirmed: bool = False
    age: float = 0.0

    @property
    def speed(self) -> float:
        return hypot(self.vx, self.vy)


@dataclass
class TrackerSettings:
    cluster_gap_m: float = 0.18
    min_points: int = 2
    max_extent_m: float = 0.8
    gate_m: float = 0.55
    gate_speed_gain: float = 0.18
    smooth_pos: float = 0.55
    smooth_vel: float = 0.35
    enter_revs: int = 2
    exit_revs: int = 4
    max_tracks: int = 24


class FootTracker:
    def __init__(self, settings: TrackerSettings | None = None) -> None:
        self.cfg = settings or TrackerSettings()
        self._tracks: dict[int, Track] = {}
        self._next_id = 1

    @property
    def tracks(self) -> list[Track]:
        return list(self._tracks.values())

    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1

    # ------------------------------------------------------------------
    @staticmethod
    def cluster_points(points: list[tuple[float, float, float]], cfg: TrackerSettings) -> list[Cluster]:
        """points: (angle, x, y) urut berdasarkan sudut."""
        if not points:
            return []

        pts = sorted(points, key=lambda p: p[0])
        groups: list[list[tuple[float, float, float]]] = []
        cur = [pts[0]]
        for prev, item in zip(pts, pts[1:]):
            # jarak antar titik berurutan (bukan jarak dari origin) -> memisahkan objek
            if hypot(item[1] - prev[1], item[2] - prev[2]) > cfg.cluster_gap_m:
                groups.append(cur)
                cur = [item]
            else:
                cur.append(item)
        groups.append(cur)

        out: list[Cluster] = []
        for g in groups:
            if len(g) < cfg.min_points:
                continue
            xs = [p[1] for p in g]
            ys = [p[2] for p in g]
            extent = hypot(max(xs) - min(xs), max(ys) - min(ys))
            if extent > cfg.max_extent_m:
                continue
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            size = max(0.04, 0.5 * extent)
            angle = sum(p[0] for p in g) / len(g)
            out.append(Cluster(x=cx, y=cy, size=size, count=len(g), angle=angle))
        return out

    # ------------------------------------------------------------------
    def update(self, points: list[tuple[float, float, float]], dt: float) -> list[Track]:
        cfg = self.cfg
        dt = max(1e-3, min(dt, 0.5))
        clusters = self.cluster_points(points, cfg)
        used: set[int] = set()

        # match terdekat (greedy)
        pairs: list[tuple[float, int, int]] = []
        for ci, c in enumerate(clusters):
            for tid, tr in self._tracks.items():
                pred_x = tr.x + tr.vx * dt
                pred_y = tr.y + tr.vy * dt
                dist = hypot(c.x - pred_x, c.y - pred_y)
                gate = cfg.gate_m + cfg.gate_speed_gain * tr.speed
                if dist <= gate:
                    pairs.append((dist, ci, tid))
        pairs.sort(key=lambda p: p[0])

        matched_tids: set[int] = set()
        for dist, ci, tid in pairs:
            if ci in used or tid in matched_tids or tid not in self._tracks:
                continue
            used.add(ci)
            matched_tids.add(tid)
            c = clusters[ci]
            tr = self._tracks[tid]
            nvx = (c.x - tr.x) / dt
            nvy = (c.y - tr.y) / dt
            tr.vx += cfg.smooth_vel * (nvx - tr.vx)
            tr.vy += cfg.smooth_vel * (nvy - tr.vy)
            tr.x += cfg.smooth_pos * (c.x - tr.x)
            tr.y += cfg.smooth_pos * (c.y - tr.y)
            tr.size += 0.25 * (c.size - tr.size)
            tr.hits += 1
            tr.misses = 0
            tr.age += dt
            if tr.hits >= cfg.enter_revs:
                tr.confirmed = True

        # cluster baru
        for ci, c in enumerate(clusters):
            if ci in used:
                continue
            if len(self._tracks) >= cfg.max_tracks:
                break
            tid = self._next_id
            self._next_id += 1
            self._tracks[tid] = Track(
                id=tid, x=c.x, y=c.y, size=c.size, hits=1, misses=0,
                confirmed=cfg.enter_revs <= 1, age=dt,
            )

        # track yang tidak dapat pasangan
        for tid in list(self._tracks.keys()):
            tr = self._tracks[tid]
            if tid not in matched_tids:
                tr.misses += 1
                tr.hits = max(0, tr.hits - 1)
                if tr.misses >= cfg.exit_revs:
                    del self._tracks[tid]
                    continue
            tr.age += dt

        return [t for t in self._tracks.values() if t.confirmed]
