"""Primitif OpenGL: batch titik instanced, mesh garis/fan, ping-pong texture."""
from __future__ import annotations

import numpy as np
import moderngl

VS_POINT = """
#version 330
uniform vec2 u_res;
in vec2 in_quad;
in vec2 in_pos;
in float in_size;
in vec4 in_color;
out vec2 v_uv;
out vec4 v_color;
void main() {
    v_uv = in_quad;
    v_color = in_color;
    vec2 p = in_pos + in_quad * in_size;
    vec2 clip = vec2(p.x / u_res.x * 2.0 - 1.0, 1.0 - p.y / u_res.y * 2.0);
    gl_Position = vec4(clip, 0.0, 1.0);
}
"""

FS_GLOW = """
#version 330
in vec2 v_uv;
in vec4 v_color;
out vec4 f_color;
uniform float u_power;
void main() {
    float d = length(v_uv);
    if (d > 1.0) discard;
    float falloff = pow(1.0 - d, u_power);
    float a = clamp(v_color.a * falloff, 0.0, 1.0);
    f_color = vec4(v_color.rgb * a, a);
}
"""

VS_FLAT = """
#version 330
uniform vec2 u_res;
in vec2 in_pos;
in vec4 in_color;
out vec4 v_color;
void main() {
    v_color = in_color;
    vec2 clip = vec2(in_pos.x / u_res.x * 2.0 - 1.0, 1.0 - in_pos.y / u_res.y * 2.0);
    gl_Position = vec4(clip, 0.0, 1.0);
}
"""

FS_FLAT = """
#version 330
in vec4 v_color;
out vec4 f_color;
void main() {
    f_color = vec4(v_color.rgb * v_color.a, v_color.a);
}
"""

VS_TEX = """
#version 330
in vec2 in_pos;
out vec2 v_uv;
uniform vec2 u_uv_scale;
uniform vec2 u_uv_offset;
void main() {
    vec2 uv = in_pos * 0.5 + 0.5;
    v_uv = uv * u_uv_scale + u_uv_offset;
    gl_Position = vec4(in_pos, 0.0, 1.0);
}
"""

FS_DECAY = """
#version 330
in vec2 v_uv;
out vec4 f_color;
uniform sampler2D u_tex;
uniform float u_decay;
void main() {
    f_color = texture(u_tex, v_uv) * u_decay;
}
"""

FS_ADD = """
#version 330
in vec2 v_uv;
out vec4 f_color;
uniform sampler2D u_tex;
uniform float u_mul;
void main() {
    vec4 c = texture(u_tex, v_uv);
    f_color = vec4(c.rgb * u_mul, c.a * u_mul);
}
"""

FS_COMPOSITE = """
#version 330
in vec2 v_uv;
out vec4 f_color;
uniform sampler2D u_tex;
uniform float u_mul;
uniform float u_invert;
uniform float u_alpha_only;
void main() {
    vec4 c = texture(u_tex, v_uv);
    float m = u_invert > 0.5 ? (1.0 - c.a) : c.a;
    if (u_alpha_only > 0.5) {
        f_color = vec4(m, m, m, 1.0);
    } else {
        f_color = vec4(c.rgb * u_mul * m, m * u_mul);
    }
}
"""


class FullscreenQuad:
    def __init__(self, ctx: moderngl.Context) -> None:
        self.ctx = ctx
        data = np.array([-1, -1, 1, -1, -1, 1, 1, 1], dtype="f4")
        self.vbo = ctx.buffer(data.tobytes())
        self.vao = ctx.vertex_array(ctx.program(vertex_shader=VS_TEX, fragment_shader=FS_ADD),
                                    [(self.vbo, "2f", "in_pos")])

    def draw(self, tex, mul: float, flip_y: bool = False) -> None:
        prog = self.vao.program
        prog["u_tex"] = 0
        prog["u_mul"] = mul
        prog["u_uv_scale"] = (1.0, -1.0 if flip_y else 1.0)
        prog["u_uv_offset"] = (0.0, 1.0 if flip_y else 0.0)
        tex.use(0)
        self.vao.render(moderngl.TRIANGLE_STRIP)


class PingPong:
    """Dua FBO RGBA untuk akumulasi trail/decay."""

    def __init__(self, ctx: moderngl.Context, w: int, h: int) -> None:
        self.ctx = ctx
        self.w, self.h = w, h
        self.tex = [self._make() for _ in range(2)]
        self.fbo = [ctx.framebuffer([t]) for t in self.tex]
        self.idx = 0
        self.quad = FullscreenQuad(ctx)
        self.decay_prog = ctx.program(vertex_shader=VS_TEX, fragment_shader=FS_DECAY)
        self.decay_vao = ctx.vertex_array(self.decay_prog, [(self.quad.vbo, "2f", "in_pos")])

    def _make(self):
        t = self.ctx.texture((self.w, self.h), 4)
        t.filter = (moderngl.LINEAR, moderngl.LINEAR)
        t.repeat_x = False
        t.repeat_y = False
        return t

    def resize(self, w: int, h: int) -> None:
        if (w, h) == (self.w, self.h):
            return
        self.w, self.h = w, h
        for t, f in zip(self.tex, self.fbo):
            t.release()
            f.release()
        self.tex = [self._make() for _ in range(2)]
        self.fbo = [self.ctx.framebuffer([t]) for t in self.tex]

    @property
    def current(self) -> moderngl.Framebuffer:
        return self.fbo[self.idx]

    @property
    def current_tex(self) -> moderngl.Texture:
        return self.tex[self.idx]

    def begin(self, decay: float, clear: bool = True) -> moderngl.Framebuffer:
        """Mulai frame baru: salin frame sebelumnya dengan decay."""
        prev_idx = self.idx
        self.idx = 1 - self.idx
        fbo = self.fbo[self.idx]
        ctx = self.ctx
        fbo.use()
        ctx.viewport = (0, 0, self.w, self.h)
        ctx.disable(moderngl.BLEND)
        if clear and decay <= 0.0:
            ctx.clear(0.0, 0.0, 0.0, 0.0)
        else:
            prog = self.decay_prog
            prog["u_tex"] = 0
            prog["u_decay"] = max(0.0, min(1.0, decay))
            self.tex[prev_idx].use(0)
            self.decay_vao.render(moderngl.TRIANGLE_STRIP)
        ctx.enable(moderngl.BLEND)
        ctx.blend_func = moderngl.ONE, moderngl.ONE
        return fbo

    def _render_quad(self, prog) -> None:
        vao = self.ctx.vertex_array(prog, [(self.quad.vbo, "2f", "in_pos")])
        try:
            vao.render(moderngl.TRIANGLE_STRIP)
        finally:
            vao.release()


class PointBatch:
    """Titik glow instanced."""

    FLOATS = 7  # pos2 + size1 + rgba4

    def __init__(self, ctx: moderngl.Context, capacity: int = 4096, power: float = 2.0) -> None:
        self.ctx = ctx
        self.capacity = capacity
        self.prog = ctx.program(vertex_shader=VS_POINT, fragment_shader=FS_GLOW)
        quad = np.array([-1, -1, 1, -1, -1, 1, 1, 1], dtype="f4")
        self.quad_buf = ctx.buffer(quad.tobytes())
        self.inst_buf = ctx.buffer(reserve=capacity * self.FLOATS * 4, dynamic=True)
        self.vao = ctx.vertex_array(
            self.prog,
            [(self.quad_buf, "2f", "in_quad"),
             (self.inst_buf, "2f 1f 4f/i", "in_pos", "in_size", "in_color")],
        )
        self.data = np.zeros((capacity, self.FLOATS), dtype="f4")
        self.n = 0
        self.power = power

    def clear(self) -> None:
        self.n = 0

    def add(self, x: float, y: float, size: float, color, alpha: float) -> None:
        if self.n >= self.capacity:
            return
        d = self.data[self.n]
        d[0] = x
        d[1] = y
        d[2] = size
        d[3] = color[0]
        d[4] = color[1]
        d[5] = color[2]
        d[6] = alpha
        self.n += 1

    def draw(self, res: tuple[float, float], power: float | None = None) -> None:
        if self.n == 0:
            return
        self.prog["u_res"] = res
        self.prog["u_power"] = self.power if power is None else power
        self.inst_buf.write(self.data[: self.n].tobytes())
        self.vao.render(moderngl.TRIANGLE_STRIP, instances=self.n)


class MeshBatch:
    """Garis/fan dengan lebar dalam pixel, dibangun per frame."""

    def __init__(self, ctx: moderngl.Context, capacity: int = 200000) -> None:
        self.ctx = ctx
        self.capacity = capacity
        self.prog = ctx.program(vertex_shader=VS_FLAT, fragment_shader=FS_FLAT)
        self.buf = ctx.buffer(reserve=capacity * 6 * 4, dynamic=True)
        self.vao = ctx.vertex_array(self.prog, [(self.buf, "2f 4f", "in_pos", "in_color")])
        self.data = np.zeros((capacity, 6), dtype="f4")
        self.n = 0  # jumlah vertex

    def clear(self) -> None:
        self.n = 0

    def _push(self, x: float, y: float, color) -> None:
        if self.n >= self.capacity:
            return
        d = self.data[self.n]
        d[0] = x
        d[1] = y
        d[2] = color[0]
        d[3] = color[1]
        d[4] = color[2]
        d[5] = color[3]
        self.n += 1

    def add_line(self, x1, y1, x2, y2, width: float, color) -> None:
        dx = x2 - x1
        dy = y2 - y1
        length = (dx * dx + dy * dy) ** 0.5
        if length < 1e-6:
            return
        nx = -dy / length * width * 0.5
        ny = dx / length * width * 0.5
        a = (x1 + nx, y1 + ny)
        b = (x1 - nx, y1 - ny)
        c = (x2 + nx, y2 + ny)
        d = (x2 - nx, y2 - ny)
        for px, py in (a, b, c, b, d, c):
            self._push(px, py, color)

    def add_polyline(self, pts, width: float, color, closed: bool = False) -> None:
        n = len(pts)
        if n < 2:
            return
        for i in range(n - 1):
            self.add_line(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], width, color)
        if closed:
            self.add_line(pts[-1][0], pts[-1][1], pts[0][0], pts[0][1], width, color)

    def add_triangle(self, p1, p2, p3, c1, c2, c3) -> None:
        for (x, y), c in ((p1, c1), (p2, c2), (p3, c3)):
            self._push(x, y, c)

    def add_fan(self, origin, arc_pts, color_center, color_edge) -> None:
        o = origin
        for i in range(len(arc_pts) - 1):
            a = arc_pts[i]
            b = arc_pts[i + 1]
            self.add_triangle(o, a, b, color_center, color_edge, color_edge)

    def draw(self, res: tuple[float, float]) -> None:
        if self.n == 0:
            return
        self.prog["u_res"] = res
        self.buf.write(self.data[: self.n].tobytes())
        self.vao.render(moderngl.TRIANGLES, vertices=self.n)
