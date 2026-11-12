"""Renderer mask gerak: blob putih di hitam, untuk trigger/mask di Resolume & TD."""
from __future__ import annotations

import moderngl
import numpy as np

from .gl import FS_COMPOSITE, VS_TEX, PingPong, PointBatch


class MaskRenderer:
    def __init__(self, ctx: moderngl.Context, width: int, height: int) -> None:
        self.ctx = ctx
        self.w = width
        self.h = height
        self.ping = PingPong(ctx, width, height)
        self.blobs = PointBatch(ctx, capacity=2048, power=1.5)
        self.out_tex = self._make_tex()
        self.out_fbo = ctx.framebuffer([self.out_tex])
        self.comp_prog = ctx.program(vertex_shader=VS_TEX, fragment_shader=FS_COMPOSITE)
        quad = np.array([-1, -1, 1, -1, -1, 1, 1, 1], dtype="f4")
        self.quad_vbo = ctx.buffer(quad.tobytes())
        self.comp_vao = ctx.vertex_array(self.comp_prog, [(self.quad_vbo, "2f", "in_pos")])

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
        self.ping.resize(width, height)

    def render(
        self,
        blobs: list[tuple[float, float, float]],
        points_px: np.ndarray,
        color,
        intensity: float,
        decay: float,
        invert: bool,
        source: str = "tracks",
    ) -> moderngl.Framebuffer:
        ctx = self.ctx
        res = (float(self.w), float(self.h))

        self.ping.begin(decay)
        ctx.viewport = (0, 0, self.w, self.h)
        self.blobs.clear()
        if source == "points":
            for x, y in points_px:
                self.blobs.add(float(x), float(y), 10.0, color, 1.0)
        else:
            for (x, y, r) in blobs:
                self.blobs.add(float(x), float(y), float(r), color, 1.0)
        self.blobs.draw(res)

        self.out_fbo.use()
        ctx.viewport = (0, 0, self.w, self.h)
        ctx.disable(moderngl.BLEND)
        prog = self.comp_prog
        prog["u_tex"] = 0
        prog["u_mul"] = intensity
        prog["u_invert"] = 1.0 if invert else 0.0
        prog["u_alpha_only"] = 1.0
        prog["u_uv_scale"] = (1.0, 1.0)
        prog["u_uv_offset"] = (0.0, 0.0)
        self.ping.current_tex.use(0)
        self.comp_vao.render(moderngl.TRIANGLE_STRIP)
        ctx.enable(moderngl.BLEND)
        ctx.blend_func = moderngl.ONE, moderngl.ONE
        return self.out_fbo
