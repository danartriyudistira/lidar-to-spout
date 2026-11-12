"""Renderer utama: titik glow, trail, sweep fan, grid, ring track."""
from __future__ import annotations

import moderngl
import numpy as np

from .gl import FullscreenQuad, MeshBatch, PingPong, PointBatch


class SceneRenderer:
    def __init__(self, ctx: moderngl.Context, width: int, height: int) -> None:
        self.ctx = ctx
        self.w = width
        self.h = height
        self.points = PointBatch(ctx, capacity=8192)
        self.mesh = MeshBatch(ctx, capacity=120000)
        self.trail = PingPong(ctx, width, height)
        self.out_tex = self._make_tex()
        self.out_fbo = ctx.framebuffer([self.out_tex])
        self.quad = FullscreenQuad(ctx)

    def _make_tex(self) -> moderngl.Texture:
        t = self.ctx.texture((self.w, self.h), 4)
        t.filter = (moderngl.LINEAR, moderngl.LINEAR)
        t.repeat_x = False
        t.repeat_y = False
        return t

    def resize(self, width: int, height: int) -> None:
        if (width, height) == (self.w, self.h):
            return
        self.w, self.h = width, height
        self.out_tex.release()
        self.out_fbo.release()
        self.out_tex = self._make_tex()
        self.out_fbo = self.ctx.framebuffer([self.out_tex])
        self.trail.resize(width, height)

    def render(
        self,
        pts_px: np.ndarray,
        color,
        point_size: float,
        point_alpha: float,
        trail_enabled: bool,
        trail_decay: float,
        trail_intensity: float,
        grid: list[tuple[float, float, float, float]] | None = None,
        grid_width: float = 2.0,
        grid_color=None,
        fan_origin: tuple[float, float] | None = None,
        fan_arc: list[tuple[float, float]] | None = None,
        fan_color=None,
        fan_alpha: float = 0.1,
        zone_poly: list[tuple[float, float]] | None = None,
        zone_color=None,
        rings: list[tuple[float, float, float]] | None = None,
        ring_color=None,
        ring_width: float = 2.5,
        bg_pts_px: np.ndarray | None = None,
        bg_color=None,
        bg_alpha: float = 0.10,
        bg_size: float = 5.0,
        lines: list | None = None,
        alphas: np.ndarray | None = None,
        raw: bool = False,
        point_power: float = 2.0,
        background: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0),
    ) -> moderngl.Framebuffer:
        ctx = self.ctx
        res = (float(self.w), float(self.h))

        def draw_points(halo: bool) -> None:
            self.points.clear()
            n = len(pts_px)
            for i in range(n):
                x, y = float(pts_px[i][0]), float(pts_px[i][1])
                a = float(alphas[i]) if alphas is not None and i < len(alphas) else point_alpha
                if a <= 0.0:
                    continue
                if halo:
                    self.points.add(x, y, point_size * 3.2, color, a * 0.18)
                self.points.add(x, y, point_size, color, a)
            self.points.draw(res, power=point_power)

        # --- 1. akumulasi trail (opsional) ---
        if trail_enabled:
            self.trail.begin(trail_decay)
            ctx.viewport = (0, 0, self.w, self.h)
            draw_points(halo=not raw)

        # --- 2. output pass ---
        self.out_fbo.use()
        ctx.viewport = (0, 0, self.w, self.h)
        ctx.clear(*background)
        ctx.enable(moderngl.BLEND)
        ctx.blend_func = moderngl.ONE, moderngl.ONE

        if trail_enabled:
            self.quad.draw(self.trail.current_tex, trail_intensity)

        self.mesh.clear()
        if grid:
            for x1, y1, x2, y2 in grid:
                self.mesh.add_line(x1, y1, x2, y2, grid_width, grid_color)
        if fan_origin and fan_arc and fan_color:
            peak = (fan_color[0], fan_color[1], fan_color[2], fan_alpha)
            edge = (fan_color[0], fan_color[1], fan_color[2], 0.0)
            self.mesh.add_fan(fan_origin, fan_arc, peak, edge)
        if zone_poly:
            self.mesh.add_polyline(zone_poly, 2.5, zone_color, closed=True)
        if lines:
            for (x1, y1, x2, y2, color, width) in lines:
                self.mesh.add_line(x1, y1, x2, y2, width, color)
        if rings:
            segs = 32
            for (cx, cy, r) in rings:
                pts = [
                    (cx + r * np.cos(2 * np.pi * i / segs), cy + r * np.sin(2 * np.pi * i / segs))
                    for i in range(segs + 1)
                ]
                self.mesh.add_polyline(pts, ring_width, ring_color)
        self.mesh.draw(res)

        # peta statis (background) digambar redup tanpa trail
        if bg_pts_px is not None and len(bg_pts_px) and bg_color is not None:
            self.points.clear()
            for x, y in bg_pts_px:
                self.points.add(float(x), float(y), bg_size, bg_color, bg_alpha)
            self.points.draw(res)

        if not trail_enabled:
            draw_points(halo=not raw)

        return self.out_fbo

    def draw_preview(self, viewport: tuple[int, int, int, int], flip_y: bool = True) -> None:
        """Gambar output ke window preview (viewport sudah dihitung app)."""
        ctx = self.ctx
        ctx.viewport = viewport
        ctx.enable(moderngl.BLEND)
        ctx.blend_func = moderngl.ONE, moderngl.ONE_MINUS_SRC_ALPHA
        self.quad.draw(self.out_tex, 1.0, flip_y=flip_y)
