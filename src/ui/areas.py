"""Model area / trigger zone (rectangle & polygon) + penyimpanan ke config."""
from __future__ import annotations

from dataclasses import dataclass, field

PALETTE = [
    [1.00, 0.55, 0.15],
    [0.20, 0.90, 0.60],
    [0.35, 0.70, 1.00],
    [1.00, 0.35, 0.60],
    [0.90, 0.85, 0.20],
    [0.70, 0.50, 1.00],
]


def slugify(text: str, fallback: str) -> str:
    out = []
    for ch in text.strip():
        out.append(ch.lower() if ch.isalnum() else "_")
    slug = "".join(out)
    slug = "_".join(part for part in slug.split("_") if part)
    return slug or fallback


def point_in_polygon(points: list[list[float]], x: float, y: float) -> bool:
    """Ray casting; works untuk rect maupun polygon (convex/concave)."""
    n = len(points)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = points[i]
        xj, yj = points[j]
        if (yi > y) != (yj > y):
            denom = (yj - yi) or 1e-12
            x_cross = (xj - xi) * (y - yi) / denom + xi
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def polygon_centroid(points: list[list[float]]) -> tuple[float, float]:
    n = len(points)
    if n == 0:
        return (0.0, 0.0)
    if n < 3:
        return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)
    area = 0.0
    cx = 0.0
    cy = 0.0
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        cross = x0 * y1 - x1 * y0
        area += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    if abs(area) < 1e-9:
        return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)
    area *= 0.5
    return (cx / (6.0 * area), cy / (6.0 * area))


@dataclass
class Area:
    id: int
    name: str
    type: str  # "rect" | "poly"
    points: list[list[float]]
    enabled: bool = True
    color: list[float] = field(default_factory=lambda: [1.0, 0.55, 0.15])
    overlap: bool = False

    @property
    def slug(self) -> str:
        return slugify(self.name, f"area_{self.id}")

    def centroid(self) -> tuple[float, float]:
        return polygon_centroid(self.points)

    def contains(self, x: float, y: float) -> bool:
        return point_in_polygon(self.points, x, y)

    def move(self, dx: float, dy: float) -> None:
        self.points = [[p[0] + dx, p[1] + dy] for p in self.points]

    def bounds(self):
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        return (min(xs), min(ys), max(xs), max(ys))

    def to_dict(self) -> dict:
        return {
            "id": int(self.id),
            "name": self.name,
            "type": self.type,
            "points": [[round(float(p[0]), 4), round(float(p[1]), 4)] for p in self.points],
            "enabled": bool(self.enabled),
            "color": [round(float(c), 3) for c in self.color],
            "overlap": bool(self.overlap),
        }

    @staticmethod
    def from_dict(d: dict) -> "Area":
        pts = [[float(p[0]), float(p[1])] for p in d.get("points", []) if len(p) >= 2]
        raw_color = [float(c) for c in d.get("color", PALETTE[0])][:3]
        while len(raw_color) < 3:
            raw_color.append(PALETTE[0][len(raw_color)])
        return Area(
            id=int(d.get("id", 0)),
            name=str(d.get("name", "Area")),
            type=str(d.get("type", "poly")),
            points=pts,
            enabled=bool(d.get("enabled", True)),
            color=raw_color,
            overlap=bool(d.get("overlap", False)),
        )


def compute_area_metrics(areas, tracks, uv_fn) -> dict[int, dict]:
    """Hitung metrik per area dari daftar track.

    areas: list[Area]; tracks: objek dengan .id/.x/.y/.vx/.vy
    uv_fn: callable (x_m, y_m) -> (u, v) ternormalisasi kanvas
    """
    metrics: dict[int, dict] = {}
    for area in areas:
        metrics[area.id] = {
            "presence": 0, "count": 0, "cx": 0.0, "cy": 0.0,
            "u": 0.0, "v": 0.0, "speed": 0.0, "ids": [],
        }
    if not areas or not tracks:
        return metrics

    buckets: dict[int, list] = {a.id: [] for a in areas}
    for t in tracks:
        hits = [a for a in areas if a.enabled and a.contains(t.x, t.y)]
        if not hits:
            continue
        if any(h.overlap for h in hits):
            chosen = hits
        else:
            chosen = [hits[-1]]  # area teratas
        for area in chosen:
            buckets[area.id].append(t)

    for area in areas:
        items = buckets[area.id]
        m = metrics[area.id]
        if not items:
            continue
        cx = sum(t.x for t in items) / len(items)
        cy = sum(t.y for t in items) / len(items)
        speed = sum((t.vx * t.vx + t.vy * t.vy) ** 0.5 for t in items) / len(items)
        u, v = uv_fn(cx, cy)
        m.update(presence=1, count=len(items), cx=cx, cy=cy,
                 u=float(u), v=float(v), speed=float(speed),
                 ids=sorted(int(t.id) for t in items))
    return metrics


class AreaStore:
    """Daftar area; semua perubahan langsung ditulis ke cfg['areas']['items']."""

    def __init__(self, cfg: dict) -> None:
        self.cfg = cfg
        self.items: list[Area] = []
        self.selected_id: int | None = None
        self._next = 1
        self.load()

    # ------------------------------------------------------------------ io
    def load(self) -> None:
        conf = self.cfg.setdefault("areas", {})
        raw = conf.get("items", [])
        self.items = [Area.from_dict(d) for d in raw if isinstance(d, dict)]
        self.items = [a for a in self.items if len(a.points) >= 3]
        self._next = max([a.id for a in self.items], default=0) + 1
        if self.selected_id not in [a.id for a in self.items]:
            self.selected_id = self.items[0].id if self.items else None

    def save(self) -> None:
        self.cfg.setdefault("areas", {})["items"] = [a.to_dict() for a in self.items]

    # ------------------------------------------------------------------ layer
    @property
    def enabled(self) -> bool:
        return bool(self.cfg.get("areas", {}).get("enabled", True))

    def set_enabled(self, value: bool) -> None:
        self.cfg.setdefault("areas", {})["enabled"] = bool(value)

    # ------------------------------------------------------------------ CRUD
    def get(self, area_id: int | None) -> Area | None:
        if area_id is None:
            return None
        for a in self.items:
            if a.id == area_id:
                return a
        return None

    @property
    def selected(self) -> Area | None:
        return self.get(self.selected_id)

    def _create(self, area_type: str, points: list[list[float]]) -> Area:
        area_id = self._next
        self._next += 1
        area = Area(id=area_id, name=f"Area {area_id}", type=area_type,
                    points=[[float(p[0]), float(p[1])] for p in points],
                    color=list(PALETTE[(area_id - 1) % len(PALETTE)]))
        self.items.append(area)
        self.selected_id = area.id
        self.save()
        return area

    def add_rect(self, p0, p1) -> Area:
        x0, x1 = sorted((float(p0[0]), float(p1[0])))
        y0, y1 = sorted((float(p0[1]), float(p1[1])))
        return self._create("rect", [[x0, y0], [x1, y0], [x1, y1], [x0, y1]])

    def add_poly(self, points) -> Area:
        return self._create("poly", [[float(x), float(y)] for x, y in points])

    def remove(self, area_id: int) -> None:
        self.items = [a for a in self.items if a.id != area_id]
        if self.selected_id == area_id:
            self.selected_id = self.items[0].id if self.items else None
        self.save()

    def duplicate(self, area_id: int) -> Area | None:
        src = self.get(area_id)
        if src is None:
            return None
        area = self._create(src.type, [list(p) for p in src.points])
        area.name = f"{src.name} copy"
        area.overlap = src.overlap
        self.save()
        return area

    def pick(self, x: float, y: float) -> Area | None:
        for area in reversed(self.items):
            if area.enabled and area.contains(x, y):
                return area
        return None

    def memberships(self, x: float, y: float) -> list[Area]:
        return [a for a in self.items if a.enabled and a.contains(x, y)]
