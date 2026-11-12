"""Editor area di viewport: rectangle drag, polygon Ctrl+klik, drag vertex/area."""
from __future__ import annotations

import math

from .areas import AreaStore


class AreaEditor:
    def __init__(self, store: AreaStore, snap_m: float = 0.25) -> None:
        self.store = store
        self.snap_m = float(snap_m)
        self.mode = "idle"          # idle | rect | poly | drag_vertex | drag_area
        self.pending: list[tuple[float, float]] = []
        self.rect_start: tuple[float, float] | None = None
        self.rect_cur: tuple[float, float] | None = None
        self.drag_vertex: tuple[int, int] | None = None
        self.drag_area: tuple[int, tuple[float, float]] | None = None
        self.unlocked = False
        self.hint = ""

    # ------------------------------------------------------------------ commands
    def arm_rect(self) -> None:
        self.mode = "rect"
        self.rect_start = None
        self.rect_cur = None
        self.pending = []
        self.hint = "Drag di preview untuk membuat rectangle · Esc batal"

    def arm_poly(self) -> None:
        self.mode = "poly"
        self.pending = []
        self.rect_start = self.rect_cur = None
        self.hint = "Klik titik sudut · klik titik awal / Enter = tutup · Backspace hapus · Esc batal"

    def cancel(self) -> None:
        self.mode = "idle"
        self.pending = []
        self.rect_start = self.rect_cur = None
        self.drag_vertex = None
        self.drag_area = None
        self.hint = ""

    def toggle_unlock(self) -> None:
        self.unlocked = not self.unlocked

    @property
    def busy(self) -> bool:
        return self.mode in ("rect", "poly", "drag_vertex", "drag_area")

    # ------------------------------------------------------------------ helpers
    def snap(self, xy, shift: bool = False):
        x, y = float(xy[0]), float(xy[1])
        if shift:
            s = self.snap_m
            x = round(x / s) * s
            y = round(y / s) * s
        return (x, y)

    def _close_poly(self) -> None:
        if len(self.pending) >= 3:
            self.store.add_poly(self.pending)
        self.mode = "idle"
        self.pending = []
        self.hint = ""

    # ------------------------------------------------------------------ events
    def on_down(self, xy, shift: bool = False) -> bool:
        """Klik kiri ditekan. Return True kalau input dipakai editor."""
        if not self.store.enabled:
            return False
        xy = self.snap(xy, shift)

        if self.mode == "rect":
            self.rect_start = xy
            self.rect_cur = xy
            return True

        if self.mode == "poly":
            if len(self.pending) >= 3:
                d = math.hypot(xy[0] - self.pending[0][0], xy[1] - self.pending[0][1])
                if d <= max(0.2, self.snap_m):
                    self._close_poly()
                    return True
            self.pending.append(xy)
            return True

        area = self.store.pick(xy[0], xy[1])
        if area is not None:
            self.store.selected_id = area.id
            self.mode = "drag_area"
            self.drag_area = (area.id, xy)
            return True

        self.store.selected_id = None
        return False

    def on_move(self, xy, shift: bool = False) -> None:
        xy = self.snap(xy, shift)
        if self.mode == "rect":
            self.rect_cur = xy
        elif self.mode == "drag_vertex" and self.drag_vertex:
            area_id, idx = self.drag_vertex
            area = self.store.get(area_id)
            if area is not None and 0 <= idx < len(area.points):
                area.points[idx] = [xy[0], xy[1]]
        elif self.mode == "drag_area" and self.drag_area:
            area_id, last = self.drag_area
            area = self.store.get(area_id)
            if area is not None:
                area.move(xy[0] - last[0], xy[1] - last[1])
                self.drag_area = (area_id, xy)

    def on_up(self, xy, shift: bool = False) -> bool:
        xy = self.snap(xy, shift)
        if self.mode == "rect" and self.rect_start is not None:
            p0 = self.rect_start
            if abs(xy[0] - p0[0]) >= 0.2 and abs(xy[1] - p0[1]) >= 0.2:
                self.store.add_rect(p0, xy)
            self.mode = "idle"
            self.rect_start = self.rect_cur = None
            self.hint = ""
            return True
        if self.mode in ("drag_vertex", "drag_area"):
            self.store.save()
            self.mode = "idle"
            self.drag_vertex = None
            self.drag_area = None
            return True
        return False

    def begin_vertex_drag(self, area_id: int, index: int) -> None:
        self.store.selected_id = area_id
        self.mode = "drag_vertex"
        self.drag_vertex = (area_id, index)

    def on_key(self, key: str, ctrl: bool = False, shift: bool = False) -> bool:
        if key == "esc":
            if self.busy:
                self.cancel()
                return True
            return False

        if self.mode == "poly":
            if key == "enter":
                self._close_poly()
                return True
            if key == "backspace":
                if self.pending:
                    self.pending.pop()
                return True

        area = self.store.selected

        if key == "delete" and area is not None:
            self.store.remove(area.id)
            return True

        if key == "d" and ctrl and area is not None:
            self.store.duplicate(area.id)
            return True

        step = 0.5 if shift else 0.1
        if key in ("left", "right", "up", "down") and area is not None:
            dx = -step if key == "left" else step if key == "right" else 0.0
            dy = -step if key == "up" else step if key == "down" else 0.0
            area.move(dx, dy)
            self.store.save()
            return True

        return False

    # ------------------------------------------------------------------ preview data
    def rect_preview(self):
        if self.mode == "rect" and self.rect_start and self.rect_cur:
            x0, y0 = self.rect_start
            x1, y1 = self.rect_cur
            return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        return None
