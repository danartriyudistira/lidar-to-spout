"""Lidar To Spout — YDLIDAR X3 -> visual scanner -> Spout (beauty + mask) + OSC."""
from __future__ import annotations

from collections import namedtuple
import argparse
import math
import sys
import time
from pathlib import Path

RoomTrack = namedtuple("RoomTrack", "id x y vx vy")

SRC_DIR = Path(__file__).resolve().parent
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import glfw
import imgui
import moderngl
import numpy as np
from imgui.integrations.glfw import GlfwRenderer

from config import load_config, save_config
from lidar.background import BackgroundModel
from lidar.geometry import crop_points, polar_to_xy
from lidar.stage import compute_canvas_size, make_mapper
from lidar.reader import ReplaySource, SerialSource
from lidar.tracker import FootTracker, TrackerSettings
from output.osc_out import OscSender
from output.spout_out import SpoutOutput
from output.tuio_out import TuioSender
from render.mask import MaskRenderer
from render.renderer import SceneRenderer
from ui.area_editor import AreaEditor
from ui.areas import AreaStore, compute_area_metrics
from ui.panel import draw_panel

APP_TITLE = "Lidar To Spout — YDLIDAR X3"
PANEL_W = 430


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


class App:
    def __init__(self, cfg_path: Path, args: argparse.Namespace) -> None:
        self.cfg_path = cfg_path
        self.args = args
        self.cfg = load_config(cfg_path)
        self._apply_cli(args)

        self.area_store = AreaStore(self.cfg)
        self.area_editor = AreaEditor(self.area_store)

        self.canvas_w, self.canvas_h = compute_canvas_size(self.cfg)
        self.fps_target = max(1, int(self.cfg["canvas"]["fps"]))

        # state
        self.tracks = []
        self.track_ids: set[int] = set()
        self.prev_track_ids: set[int] = set()
        self._rev_idx = -1
        self._rev_time = time.perf_counter()
        self.fps = 0.0
        self.points_total = 0
        self.frame_count = 0
        self.running = True
        self.show_panel = bool(self.cfg["ui"].get("show_panel", True))
        self.show_preview = not args.headless
        self.edit_quad = False
        self._sweep_angle = None
        self._drag_corner = -1
        self._drag_lidar = 0        # 0=none, 1=geser posisi, 2=putar heading
        self._mouse_prev = False
        self._keys: dict[int, bool] = {}
        self._mapper_sig = None
        self._resize_at = 0.0
        self._resize_to: tuple[int, int] | None = None
        self._osc_sig = None
        self._tuio_sig = None
        self._filter_sig = None
        self.area_metrics: dict[int, dict] = {}
        self._area_prev_ids: dict[int, set[int]] = {}
        self._area_events: list[tuple[str, int, int]] = []

        self._init_window()
        self._init_gl()
        self._init_pipeline()

    # ------------------------------------------------------------------ setup
    def _apply_cli(self, args: argparse.Namespace) -> None:
        if args.port:
            self.cfg["serial"]["port"] = args.port
        if args.baud:
            self.cfg["serial"]["baud"] = args.baud
        if args.replay:
            self.cfg["serial"]["replay_file"] = args.replay
        if args.speed:
            self.cfg["serial"]["replay_speed"] = args.speed
        if args.width:
            self.cfg["canvas"]["width"] = args.width
            self.cfg["stage"]["resolution"]["mode"] = "fixed"
        if args.height:
            self.cfg["canvas"]["height"] = args.height
            self.cfg["stage"]["resolution"]["mode"] = "fixed"
        if args.fps:
            self.cfg["canvas"]["fps"] = args.fps
        if args.no_spout:
            self.cfg["spout"]["enabled"] = False
        if args.no_osc:
            self.cfg["osc"]["enabled"] = False
        if args.no_tuio:
            self.cfg["tuio"]["enabled"] = False
        if args.no_trail:
            self.cfg["style"]["trail_enabled"] = False

    def _init_window(self) -> None:
        if not glfw.init():
            raise SystemExit("Gagal init GLFW")
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        glfw.window_hint(glfw.DOUBLEBUFFER, glfw.TRUE)
        visible = not self.args.headless
        glfw.window_hint(glfw.VISIBLE, glfw.TRUE if visible else glfw.FALSE)
        w = int(self.cfg["ui"].get("preview_width", 1280))
        h = int(self.cfg["ui"].get("preview_height", 360))
        self.win = glfw.create_window(w, h, APP_TITLE, None, None)
        if not self.win:
            raise SystemExit("Gagal membuat window OpenGL")
        glfw.make_context_current(self.win)
        glfw.swap_interval(0)

    def _init_gl(self) -> None:
        self.ctx = moderngl.create_context()
        self.ctx.gc_mode = "auto"
        print(f"[gl] {self.ctx.info['GL_RENDERER']} | {self.ctx.info['GL_VERSION']}")
        imgui.create_context()
        self.impl = GlfwRenderer(self.win)
        io = imgui.get_io()
        io.ini_file_name = None

    def _init_pipeline(self) -> None:
        cfg = self.cfg
        self.scene = SceneRenderer(self.ctx, self.canvas_w, self.canvas_h)
        self.mask = MaskRenderer(self.ctx, self.canvas_w, self.canvas_h)

        spout_cfg = cfg["spout"]
        self.spout_beauty = SpoutOutput(
            spout_cfg.get("beauty", "Lidar Beauty"), self.canvas_w, self.canvas_h,
            invert=bool(spout_cfg.get("invert", True)), enabled=bool(spout_cfg.get("enabled", True)),
        )
        self.spout_mask = SpoutOutput(
            spout_cfg.get("mask", "Lidar Mask"), self.canvas_w, self.canvas_h,
            invert=bool(spout_cfg.get("invert", True)),
            enabled=bool(spout_cfg.get("enabled", True)) and bool(cfg["mask"].get("enabled", True)),
        )

        osc_cfg = cfg["osc"]
        self.osc = OscSender(osc_cfg.get("host", "127.0.0.1"), osc_cfg.get("port", 9000),
                             enabled=bool(osc_cfg.get("enabled", True)))
        self._osc_interval = 1.0 / max(1.0, float(osc_cfg.get("rate_hz", 30)))
        self._osc_last = 0.0
        self._osc_sig = (bool(osc_cfg.get("enabled", True)), str(osc_cfg.get("host", "127.0.0.1")),
                         int(osc_cfg.get("port", 9000)), float(osc_cfg.get("rate_hz", 30)))

        tuio_cfg = cfg["tuio"]
        self.tuio = TuioSender(tuio_cfg.get("host", "127.0.0.1"), tuio_cfg.get("port", 3333),
                               enabled=bool(tuio_cfg.get("enabled", True)))
        self._tuio_interval = 1.0 / max(1.0, float(tuio_cfg.get("rate_hz", 30)))
        self._tuio_last = 0.0
        self._tuio_sig = (bool(tuio_cfg.get("enabled", True)), str(tuio_cfg.get("host", "127.0.0.1")),
                          int(tuio_cfg.get("port", 3333)), float(tuio_cfg.get("rate_hz", 30)))

        self.mapping = make_mapper(cfg, self.canvas_w, self.canvas_h, self._output_crop_rect())
        self._mapper_sig = self._stage_signature()
        f = cfg["filter"]
        self.background = BackgroundModel(int(f.get("bins", 720)), float(f.get("tol_m", 0.12)),
                                          int(f.get("learn_revs", 3)))
        self._filter_sig = (int(f.get("bins", 720)), int(f.get("learn_revs", 3)))
        self._learn_until = 0.0
        if f.get("enabled", True) and f.get("auto_learn", True):
            self._learn_until = time.perf_counter() + float(f.get("learn_seconds", 4.0))
            self.background.learning = True
        self._rebuild_tracker()
        self._start_source()

    def _start_source(self) -> None:
        if getattr(self, "source", None) is not None:
            self.source.stop()
            self.source = None
        cfg = self.cfg["serial"]
        replay = cfg.get("replay_file") or ""
        if replay:
            path = Path(replay)
            if not path.is_absolute() and not path.exists():
                base = (Path(sys.executable).resolve().parent
                        if getattr(sys, "frozen", False) else SRC_DIR.parent)
                candidate = base / replay
                if candidate.exists():
                    path = candidate
            self.source = ReplaySource(path, baud=int(cfg.get("baud", 115200)),
                                       speed=float(cfg.get("replay_speed", 1.0)), loop=True)
            print(f"[source] replay {path} @ {cfg.get('replay_speed', 1.0)}x")
        else:
            self.source = SerialSource(cfg.get("port", "COM4"), int(cfg.get("baud", 115200)))
            print(f"[source] serial {cfg.get('port')} @ {cfg.get('baud')}")
        self.source.start()

    def _rebuild_tracker(self) -> None:
        t = self.cfg["tracker"]
        self.tracker = FootTracker(TrackerSettings(
            cluster_gap_m=float(t.get("cluster_gap_m", 0.18)),
            min_points=int(t.get("min_points", 2)),
            max_extent_m=float(t.get("max_extent_m", 0.8)),
            gate_m=float(t.get("gate_m", 0.55)),
            smooth_pos=float(t.get("smooth_pos", 0.55)),
            smooth_vel=float(t.get("smooth_vel", 0.35)),
            enter_revs=int(t.get("enter_revs", 2)),
            exit_revs=int(t.get("exit_revs", 4)),
            max_tracks=int(t.get("max_tracks", 24)),
        ))

    # ------------------------------------------------------------------ helpers
    def _stage_signature(self):
        st = self.cfg["stage"]
        room = st.get("room", {})
        lidar = st.get("lidar", {})
        cv = st.get("canvas", {})
        quad = np.asarray(self.cfg["map"]["quad"], dtype=np.float64).reshape(-1)
        crop = self._output_crop_rect()
        crop_sig = tuple(round(float(v), 3) for v in crop) if crop else ()
        return (
            bool(st.get("enabled", True)),
            round(float(room.get("width_m", 8.0)), 4),
            round(float(room.get("height_m", 4.0)), 4),
            round(float(lidar.get("x_m", 0.0)), 4),
            round(float(lidar.get("y_m", 0.0)), 4),
            round(float(lidar.get("heading_deg", 0.0)), 3),
            bool(lidar.get("mirror_y", False)),
            str(cv.get("aspect", "16:9")),
            round(float(cv.get("aspect_custom", 2.0)), 4),
            str(cv.get("fit", "stretch")),
            tuple(round(float(v), 2) for v in quad),
            self.canvas_w,
            self.canvas_h,
            crop_sig,
        )

    def _output_crop_rect(self):
        """Bounding box area yang dijadikan output Spout (None = seluruh ruang)."""
        aid = int(self.cfg.get("areas", {}).get("output_area", 0) or 0)
        if not aid:
            return None
        area = self.area_store.get(aid)
        if area is None or not area.enabled or len(area.points) < 3:
            return None
        return area.bounds()

    def _sync_stage(self, now: float) -> None:
        """Rebuild mapper saat parameter berubah; resize kanvas dengan debounce."""
        want_w, want_h = compute_canvas_size(self.cfg)
        if (want_w, want_h) != (self.canvas_w, self.canvas_h):
            if self._resize_to != (want_w, want_h):
                self._resize_to = (want_w, want_h)
                self._resize_at = now + 0.3
            elif now >= self._resize_at:
                self._apply_canvas(want_w, want_h)
                self._resize_to = None
        else:
            self._resize_to = None

        sig = self._stage_signature()
        if sig != self._mapper_sig and not (self.area_editor.busy or self._drag_lidar):
            self._mapper_sig = sig
            self.mapping = make_mapper(self.cfg, self.canvas_w, self.canvas_h, self._output_crop_rect())

    def _apply_canvas(self, w: int, h: int) -> None:
        w = max(64, int(w))
        h = max(64, int(h))
        if (w, h) == (self.canvas_w, self.canvas_h):
            return
        self.canvas_w, self.canvas_h = w, h
        print(f"[canvas] resolusi kanvas {w}x{h}")
        self.scene.resize(w, h)
        self.mask.resize(w, h)
        self.spout_beauty.resize(w, h)
        self.spout_mask.resize(w, h)
        self.mapping = make_mapper(self.cfg, w, h, self._output_crop_rect())
        self._mapper_sig = self._stage_signature()

    def _polar_to_canvas(self, angles: np.ndarray, radius: float) -> np.ndarray:
        world = np.array([polar_to_xy(float(a), radius) for a in angles], dtype=np.float64)
        return self.mapping.world_to_canvas(world)

    def _crop(self, pts) -> list[tuple[float, float, float]]:
        c = self.cfg["crop"]
        return crop_points(pts, float(c["angle_min"]), float(c["angle_max"]),
                           float(c["dist_min"]), float(c["dist_max"]),
                           bool(c.get("use_angle", True)))

    def _px_per_m(self) -> float:
        sx, sy = self.mapping.px_per_m()
        return float(max(1e-6, (sx + sy) * 0.5))

    def _grid_segments(self) -> list[tuple[float, float, float, float]]:
        st = self.cfg["style"]
        if not st.get("grid_enabled", True):
            return []
        return self.mapping.grid_segments(max(0.1, float(st.get("grid_step_m", 1.0))))

    def _overlay_lines(self):
        """Garis tambahan: outline ruang + marker posisi LiDAR + arah heading."""
        lines = []
        st = self.cfg["style"]
        outline = self.mapping.room_outline()
        if st.get("room_outline", True) and len(outline) > 1:
            col = (*st.get("room_color", [0.35, 0.55, 0.65]), 0.5)
            for a, b in zip(outline, outline[1:]):
                lines.append((a[0], a[1], b[0], b[1], col, 2.0))
        if self.cfg.get("areas", {}).get("show_in_output", False) and self.area_store.enabled:
            for area in self.area_store.items:
                if not area.enabled or len(area.points) < 3:
                    continue
                pts = self.mapping.room_to_canvas(area.points)
                col = (area.color[0], area.color[1], area.color[2], 0.55)
                for a, b in zip(list(pts) + [pts[0]], list(pts)[1:] + [pts[0]]):
                    lines.append((a[0], a[1], b[0], b[1], col, 2.0))
        marker = self.mapping.lidar_marker()
        if marker and st.get("lidar_marker", True):
            pos, fwd = marker
            col = (*st.get("marker_color", [1.0, 0.8, 0.25]), 0.9)
            r = 14.0
            lines.append((pos[0] - r, pos[1], pos[0] + r, pos[1], col, 2.0))
            lines.append((pos[0], pos[1] - r, pos[0], pos[1] + r, col, 2.0))
            dx, dy = fwd[0] - pos[0], fwd[1] - pos[1]
            ln = math.hypot(dx, dy) or 1.0
            lines.append((pos[0], pos[1], pos[0] + dx / ln * 44.0, pos[1] + dy / ln * 44.0, col, 3.0))
        return lines

    def _zone_poly(self):
        c = self.cfg["crop"]
        if not bool(c.get("use_angle", True)):
            return None
        a0 = float(c["angle_min"])
        a1 = float(c["angle_max"])
        span = (a1 - a0) % 360.0
        if span < 1e-3:
            span = 360.0
        n = max(8, int(span / 4))
        angs = np.linspace(a0, a0 + span, n)
        pts = self._polar_to_canvas(angs, float(c["dist_max"]))
        if span >= 359.5:
            return [tuple(p) for p in pts]
        o = self._polar_to_canvas(np.array([a0, a0 + span]), 0.0)
        return [tuple(o[0])] + [tuple(p) for p in pts] + [tuple(o[1])]

    def _fan(self):
        st = self.cfg["style"]
        if not st.get("sweep_enabled", True) or self._sweep_angle is None:
            return None, None
        width = float(st.get("sweep_width_deg", 55.0))
        angs = np.linspace(self._sweep_angle - width * 0.5, self._sweep_angle + width * 0.5, 20)
        radius = float(self.cfg["crop"]["dist_max"])
        arc = self._polar_to_canvas(angs, radius)
        origin = self._polar_to_canvas(np.array([self._sweep_angle]), 0.0)[0]
        return tuple(origin), [tuple(p) for p in arc]

    # ------------------------------------------------------------------ frame
    def _handle_keys(self) -> None:
        try:
            if imgui.get_io().want_text_input:
                return  # sedang mengetik di field panel
        except Exception:
            pass

        def pressed(key: int) -> bool:
            down = glfw.get_key(self.win, key) == glfw.PRESS
            was = self._keys.get(key, False)
            self._keys[key] = down
            return down and not was

        ctrl = (glfw.get_key(self.win, glfw.KEY_LEFT_CONTROL) == glfw.PRESS
                or glfw.get_key(self.win, glfw.KEY_RIGHT_CONTROL) == glfw.PRESS)
        shift = (glfw.get_key(self.win, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                 or glfw.get_key(self.win, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)

        editor_keys = {
            glfw.KEY_DELETE: "delete",
            glfw.KEY_ENTER: "enter",
            glfw.KEY_KP_ENTER: "enter",
            glfw.KEY_ESCAPE: "esc",
            glfw.KEY_BACKSPACE: "backspace",
            glfw.KEY_LEFT: "left",
            glfw.KEY_RIGHT: "right",
            glfw.KEY_UP: "up",
            glfw.KEY_DOWN: "down",
            glfw.KEY_D: "d",
        }
        for key, name in editor_keys.items():
            if pressed(key) and self.area_editor.on_key(name, ctrl, shift):
                return

        if pressed(glfw.KEY_A):
            self.area_editor.toggle_unlock()
        if pressed(glfw.KEY_H):
            self.show_panel = not self.show_panel
        if pressed(glfw.KEY_P):
            self.show_preview = not self.show_preview
        if pressed(glfw.KEY_Q):
            self.edit_quad = not self.edit_quad
        if pressed(glfw.KEY_S):
            save_config(self.cfg_path, self.cfg)
            print(f"[config] disimpan ke {self.cfg_path}")
        if pressed(glfw.KEY_ESCAPE):
            self.running = False

    def _preview_rect(self, fb_w: int, fb_h: int) -> tuple[int, int, int, int]:
        """Rect preview dalam koordinat framebuffer, origin top-left."""
        aspect = self.canvas_w / self.canvas_h
        if fb_w / max(1, fb_h) > aspect:
            h = fb_h
            w = int(h * aspect)
        else:
            w = fb_w
            h = int(w / aspect)
        return (fb_w - w) // 2, (fb_h - h) // 2, max(1, w), max(1, h)

    def _quad_norm(self) -> np.ndarray:
        """Quad dalam koordinat ternormalisasi 0..1 (kompatibel quad piksel lama)."""
        q = np.asarray(self.cfg["map"]["quad"], dtype=np.float64).reshape(4, 2)
        if float(np.min(q)) >= -0.5 and float(np.max(q)) <= 1.5:
            return q.copy()
        ref_w = max(1.0, float(self.cfg["canvas"].get("width", self.canvas_w)))
        ref_h = max(1.0, float(self.cfg["canvas"].get("height", self.canvas_h)))
        return q / np.array([ref_w, ref_h])

    def _preview_rect_window(self) -> tuple[int, int, int, int]:
        win_w, win_h = glfw.get_window_size(self.win)
        if win_w <= 0 or win_h <= 0:
            return (0, 0, 1, 1)
        aspect = self.canvas_w / max(1, self.canvas_h)
        if win_w / max(1, win_h) > aspect:
            h = win_h
            w = int(h * aspect)
        else:
            w = win_w
            h = int(w / aspect)
        return (win_w - w) // 2, (win_h - h) // 2, max(1, w), max(1, h)

    def _draw_area_overlay(self) -> None:
        """Overlay area (hanya di preview, tidak masuk output Spout)."""
        rx, ry, rw, rh = self._preview_rect_window()
        dl = imgui.get_background_draw_list()

        def col(r, g, b, a):
            return imgui.get_color_u32_rgba(float(r), float(g), float(b), float(a))

        def to_win(pts):
            arr = self.mapping.room_to_canvas(pts)
            return [(rx + p[0] / self.canvas_w * rw, ry + p[1] / self.canvas_h * rh) for p in arr]

        def fill_poly(pts, color):
            if len(pts) < 3:
                return
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            for i in range(len(pts)):
                a = pts[i]
                b = pts[(i + 1) % len(pts)]
                dl.add_triangle_filled(cx, cy, a[0], a[1], b[0], b[1], color)

        store = self.area_store
        editor = self.area_editor
        metrics = self.area_metrics
        if store.enabled:
            for i, area in enumerate(store.items):
                if len(area.points) < 3:
                    continue
                pts = to_win(area.points)
                selected = area.id == store.selected_id
                count = int(metrics.get(area.id, {}).get("count", 0))
                alpha = 1.0 if area.enabled else 0.35
                color = col(area.color[0], area.color[1], area.color[2], alpha)
                if area.enabled:
                    fill_alpha = 0.18 if selected else 0.08
                    if count:
                        fill_alpha = min(0.35, fill_alpha + 0.10)
                    fill_poly(pts, col(area.color[0], area.color[1], area.color[2], fill_alpha))
                dl.add_polyline(pts + [pts[0]], color, 0, 3.0 if selected else 2.0)
                label_pos = to_win([list(area.centroid())])[0]
                label = f"{i + 1}. {area.name}"
                if count:
                    label += f"  ({count})"
                dl.add_text(label_pos[0] + 6, label_pos[1] - 7, color, label)
                if selected and editor.unlocked:
                    for p in pts:
                        dl.add_rect_filled(p[0] - 4, p[1] - 4, p[0] + 4, p[1] + 4, col(1, 1, 1, 0.95))
                        dl.add_rect(p[0] - 4, p[1] - 4, p[0] + 4, p[1] + 4, color, 0.0, 0, 1.0)

            if editor.mode == "rect" and editor.rect_preview():
                pts = to_win(editor.rect_preview())
                dl.add_polyline(pts + [pts[0]], col(1, 1, 1, 0.85), 0, 2.0)
            if editor.mode == "poly" and editor.pending:
                pts = to_win(editor.pending)
                dl.add_polyline(pts, col(1, 1, 1, 0.9), 0, 2.0)
                mx, my = imgui.get_io().mouse_pos
                dl.add_line(pts[-1][0], pts[-1][1], mx, my, col(1, 1, 1, 0.45), 1.5)
                for p in pts:
                    dl.add_circle_filled(p[0], p[1], 3.0, col(1, 1, 1, 0.9))

        if editor.hint:
            dl.add_text(rx + 12, ry + 10, col(1.0, 0.9, 0.5, 0.95), editor.hint)

    def _handle_quad_drag(self, fb_w: int, fb_h: int) -> None:
        if not self.edit_quad:
            self._drag_corner = -1
            return
        if self.impl.io.want_capture_mouse:
            return
        mx, my = glfw.get_cursor_pos(self.win)
        win_w, win_h = glfw.get_window_size(self.win)
        sx = fb_w / max(1, win_w)
        sy = fb_h / max(1, win_h)
        mx, my = mx * sx, my * sy
        rx, ry, rw, rh = self._preview_rect(fb_w, fb_h)

        quad = self._quad_norm()
        canvas_pts = [(rx + u * rw, ry + v * rh) for u, v in quad]

        down = glfw.get_mouse_button(self.win, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        if down and not self._mouse_prev:
            best, best_d = -1, 1e9
            for i, (px, py) in enumerate(canvas_pts):
                d = math.hypot(px - mx, py - my)
                if d < best_d:
                    best, best_d = i, d
            if best_d < 40:
                self._drag_corner = best
        if not down:
            self._drag_corner = -1
        if down and self._drag_corner >= 0:
            u = (mx - rx) / max(1, rw)
            v = (my - ry) / max(1, rh)
            self.cfg["map"]["quad"][self._drag_corner] = [float(u), float(v)]
            self._mapper_sig = None
        self._mouse_prev = down

    def _cursor_canvas(self, fb_w: int, fb_h: int):
        mx, my = glfw.get_cursor_pos(self.win)
        win_w, win_h = glfw.get_window_size(self.win)
        mx *= fb_w / max(1, win_w)
        my *= fb_h / max(1, win_h)
        rx, ry, rw, rh = self._preview_rect(fb_w, fb_h)
        inside = rx <= mx <= rx + rw and ry <= my <= ry + rh
        cx = (mx - rx) / max(1, rw) * self.canvas_w
        cy = (my - ry) / max(1, rh) * self.canvas_h
        return (float(cx), float(cy)), inside

    def _handle_preview_input(self, fb_w: int, fb_h: int) -> None:
        down = glfw.get_mouse_button(self.win, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        tried = down and not self._mouse_prev
        try:
            (cx, cy), inside = self._cursor_canvas(fb_w, fb_h)
            room_arr = self.mapping.canvas_to_room([[cx, cy]])[0]
            room = (float(room_arr[0]), float(room_arr[1]))
            alt = (glfw.get_key(self.win, glfw.KEY_LEFT_ALT) == glfw.PRESS
                   or glfw.get_key(self.win, glfw.KEY_RIGHT_ALT) == glfw.PRESS)
            shift = (glfw.get_key(self.win, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                     or glfw.get_key(self.win, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
            ctrl = (glfw.get_key(self.win, glfw.KEY_LEFT_CONTROL) == glfw.PRESS
                    or glfw.get_key(self.win, glfw.KEY_RIGHT_CONTROL) == glfw.PRESS)

            if self.impl.io.want_capture_mouse:
                tried = False  # jangan mulai interaksi baru saat kursor di panel

            consumed = False
            if inside or self._mouse_prev:
                consumed = self._handle_area_input(room, (cx, cy), down, tried, ctrl, shift)
            if (inside or not down) and not consumed and not self.area_editor.busy:
                self._handle_marker_drag(room, (cx, cy), down, tried, alt, shift)
        finally:
            self._mouse_prev = down

    def _handle_area_input(self, room, px, down: bool, tried: bool, ctrl: bool, shift: bool) -> bool:
        store = self.area_store
        editor = self.area_editor
        if not store.enabled:
            return False

        hover = None
        area = store.selected
        if editor.unlocked and area is not None and len(area.points) >= 3:
            pts = self.mapping.room_to_canvas(area.points)
            best = 12.0
            for i, p in enumerate(pts):
                d = math.hypot(p[0] - px[0], p[1] - px[1])
                if d < best:
                    best, hover = d, i

        consumed = False
        if tried:
            if hover is not None:
                editor.begin_vertex_drag(area.id, hover)
                consumed = True
            else:
                consumed = editor.on_down(room, shift)
                if not consumed and ctrl and editor.mode == "idle":
                    editor.arm_poly()
                    consumed = editor.on_down(room, shift)
        if down:
            editor.on_move(room, shift)
        if not down and self._mouse_prev:
            consumed = editor.on_up(room, shift) or consumed
        return consumed

    def _handle_marker_drag(self, room, px, down: bool, tried: bool, alt: bool, shift: bool) -> None:
        if not self.mapping.is_stage:
            return
        marker = self.mapping.lidar_marker()
        if not marker:
            return
        pos, _ = marker
        if tried and math.hypot(pos[0] - px[0], pos[1] - px[1]) <= 30:
            self._drag_lidar = 2 if alt else 1
        if not down:
            self._drag_lidar = 0
        if not (down and self._drag_lidar):
            return
        st = self.cfg["stage"]
        lidar = st["lidar"]
        if self._drag_lidar == 1:
            x, y = float(room[0]), float(room[1])
            if shift:
                x = round(x / 0.25) * 0.25
                y = round(y / 0.25) * 0.25
            lidar["x_m"] = float(min(max(0.0, x), float(st["room"]["width_m"])))
            lidar["y_m"] = float(min(max(0.0, y), float(st["room"]["height_m"])))
        else:
            lx = float(lidar.get("x_m", 0.0))
            ly = float(lidar.get("y_m", 0.0))
            ang = math.degrees(math.atan2(room[1] - ly, room[0] - lx))
            if shift:
                ang = round(ang / 5.0) * 5.0
            lidar["heading_deg"] = float(ang)
        self._mapper_sig = None

    # ------------------------------------------------------------------ area metrics
    def _uv(self, x: float, y: float) -> tuple[float, float]:
        p = self.mapping.room_to_canvas([[x, y]])[0]
        return (float(p[0] / max(1, self.canvas_w)), float(p[1] / max(1, self.canvas_h)))

    def _update_area_metrics(self) -> None:
        store = self.area_store
        if not store.enabled:
            self.area_metrics = {}
            self._area_prev_ids.clear()
            return
        room_tracks = []
        if self.tracks:
            xy = self.mapping.lidar_to_room([[t.x, t.y] for t in self.tracks])
            for t, p in zip(self.tracks, xy):
                vx, vy = self.mapping.lidar_vec_to_room(t.vx, t.vy)
                room_tracks.append(RoomTrack(int(t.id), float(p[0]), float(p[1]), float(vx), float(vy)))
        metrics = compute_area_metrics(store.items, room_tracks, self._uv)
        for area in store.items:
            m = metrics.get(area.id)
            if m is None:
                continue
            current = set(m["ids"])
            previous = self._area_prev_ids.get(area.id, set())
            for tid in current - previous:
                self._area_events.append((area.slug, int(tid), 1))
            for tid in previous - current:
                self._area_events.append((area.slug, int(tid), 0))
            self._area_prev_ids[area.id] = current
        self.area_metrics = metrics

    def _sync_outputs(self) -> None:
        o = self.cfg["osc"]
        sig = (bool(o.get("enabled", True)), str(o.get("host", "127.0.0.1")),
               int(o.get("port", 9000)), float(o.get("rate_hz", 30)))
        if sig != self._osc_sig:
            self._osc_sig = sig
            try:
                self.osc.close()
            except Exception:
                pass
            self.osc = OscSender(sig[1], sig[2], enabled=sig[0])
            self._osc_interval = 1.0 / max(1.0, sig[3])

        t = self.cfg["tuio"]
        tsig = (bool(t.get("enabled", True)), str(t.get("host", "127.0.0.1")),
                int(t.get("port", 3333)), float(t.get("rate_hz", 30)))
        if tsig != self._tuio_sig:
            self._tuio_sig = tsig
            try:
                self.tuio.close()
            except Exception:
                pass
            self.tuio = TuioSender(tsig[1], tsig[2], enabled=tsig[0])
            self._tuio_interval = 1.0 / max(1.0, tsig[3])

    def _sync_tracker_settings(self) -> None:
        t = self.cfg["tracker"]
        s = self.tracker.cfg
        s.cluster_gap_m = float(t.get("cluster_gap_m", s.cluster_gap_m))
        s.min_points = int(t.get("min_points", s.min_points))
        s.max_extent_m = float(t.get("max_extent_m", s.max_extent_m))
        s.gate_m = float(t.get("gate_m", s.gate_m))
        s.smooth_pos = float(t.get("smooth_pos", s.smooth_pos))
        s.smooth_vel = float(t.get("smooth_vel", s.smooth_vel))
        s.enter_revs = int(t.get("enter_revs", s.enter_revs))
        s.exit_revs = int(t.get("exit_revs", s.exit_revs))
        s.max_tracks = int(t.get("max_tracks", s.max_tracks))

    def _sync_filter(self) -> None:
        f = self.cfg["filter"]
        sig = (int(f.get("bins", 720)), int(f.get("learn_revs", 3)))
        if sig == self._filter_sig:
            return
        self._filter_sig = sig
        self.background = BackgroundModel(sig[0], float(f.get("tol_m", 0.12)), sig[1])
        if f.get("enabled", True) and f.get("auto_learn", True):
            self._learn_until = time.perf_counter() + float(f.get("learn_seconds", 4.0))
            self.background.learning = True

    def _run_tracker(self, cropped_rev, now: float) -> None:
        if not self.cfg["tracker"].get("enabled", True):
            self.tracks = []
            return
        dt = max(1e-3, now - self._rev_time)
        self._rev_time = now
        dt = min(dt, 0.5)
        self.tracks = self.tracker.update(cropped_rev, dt)
        self.track_ids = {t.id for t in self.tracks}

    def _publish_osc(self) -> None:
        if not self.osc.enabled:
            self._area_events.clear()
            return
        now = time.monotonic()
        if now - self._osc_last < self._osc_interval:
            return
        self._osc_last = now
        for tid in self.track_ids - self.prev_track_ids:
            self.osc.send("/zone/enter", int(tid))
        for tid in self.prev_track_ids - self.track_ids:
            self.osc.send("/zone/exit", int(tid))
        self.prev_track_ids = set(self.track_ids)
        self.osc.send("/zone/presence", 1 if self.tracks else 0)
        self.osc.send("/zone/count", int(len(self.tracks)))
        for t in self.tracks:
            px = self.mapping.world_to_canvas(np.array([[t.x, t.y]]))[0]
            self.osc.send(f"/foot/{t.id}/x", float(t.x))
            self.osc.send(f"/foot/{t.id}/y", float(t.y))
            self.osc.send(f"/foot/{t.id}/u", float(px[0] / self.canvas_w))
            self.osc.send(f"/foot/{t.id}/v", float(px[1] / self.canvas_h))
            self.osc.send(f"/foot/{t.id}/vx", float(t.vx))
            self.osc.send(f"/foot/{t.id}/vy", float(t.vy))
            self.osc.send(f"/foot/{t.id}/size", float(t.size))
            self.osc.send(f"/foot/{t.id}/active", 1)

        if self.area_store.enabled:
            for area in self.area_store.items:
                m = self.area_metrics.get(area.id)
                if not m:
                    continue
                base = f"/area/{area.slug}"
                self.osc.send(base + "/presence", int(m["presence"]))
                self.osc.send(base + "/count", int(m["count"]))
                self.osc.send(base + "/cx", float(m["cx"]))
                self.osc.send(base + "/cy", float(m["cy"]))
                self.osc.send(base + "/u", float(m["u"]))
                self.osc.send(base + "/v", float(m["v"]))
                self.osc.send(base + "/speed", float(m["speed"]))
        for slug, tid, kind in self._area_events:
            self.osc.send(f"/area/{slug}/{'enter' if kind else 'exit'}", int(tid))
        self._area_events.clear()

    def _room_size(self) -> tuple[float, float]:
        if self.mapping.is_stage:
            st = self.cfg["stage"]["room"]
            return (max(0.1, float(st["width_m"])), max(0.1, float(st["height_m"])))
        x0, y0, x1, y1 = self.cfg["map"]["world_rect"]
        return (max(0.1, abs(float(x1) - float(x0))), max(0.1, abs(float(y1) - float(y0))))

    def _publish_tuio(self) -> None:
        if not self.tuio.enabled:
            return
        now = time.monotonic()
        if now - self._tuio_last < self._tuio_interval:
            return
        self._tuio_last = now
        tracks = self.tracks
        if not tracks:
            self.tuio.send([])
            return

        room = self.mapping.lidar_to_room([[t.x, t.y] for t in tracks])
        room_w, room_h = self._room_size()

        allowed = None
        filt = str(self.cfg["tuio"].get("area_filter", "")).strip().lower()
        if filt:
            area = next((a for a in self.area_store.items if a.slug == filt), None)
            if area is not None:
                allowed = set(self.area_metrics.get(area.id, {}).get("ids", []))

        cursors = []
        for t, p in zip(tracks, room):
            if allowed is not None and int(t.id) not in allowed:
                continue
            u, v = self._uv(float(p[0]), float(p[1]))
            vx, vy = self.mapping.lidar_vec_to_room(t.vx, t.vy)
            speed_norm = math.hypot(vx / room_w, vy / room_h)
            cursors.append((int(t.id), u, v, speed_norm, float(t.size / room_w)))
        self.tuio.send(cursors)

    def _render_frame(self, snap) -> None:
        cfg = self.cfg
        style = cfg["style"]
        spout_on = bool(cfg["spout"].get("enabled", True))
        self.spout_beauty.set_enabled(spout_on)
        self.spout_mask.set_enabled(spout_on and bool(cfg["mask"].get("enabled", True)))

        cropped_rev = self._crop(snap.rev_points)
        cropped_live = self._crop(snap.live_points)
        now = time.perf_counter()
        self._sync_stage(now)
        self._sync_outputs()
        self._sync_tracker_settings()
        self._sync_filter()
        flt = cfg["filter"]
        new_rev = snap.rev_index != self._rev_idx
        if new_rev:
            self._rev_idx = snap.rev_index
            if flt.get("enabled", True):
                self.background.tol = float(flt.get("tol_m", 0.12))
                if now < self._learn_until:
                    self.background.learning = True
                    self.background.update(cropped_rev)
                else:
                    self.background.learning = False
        if flt.get("enabled", True) and not flt.get("show_all", False):
            cropped_rev, bg_rev = self.background.split(cropped_rev)
            cropped_live, _ = self.background.split(cropped_live)
            if flt.get("show_background", True):
                bg_world = [(p[1], p[2]) for p in bg_rev]
            else:
                bg_world = []
        else:
            bg_world = []
        if new_rev:
            self._run_tracker(cropped_rev, now)
        self._update_area_metrics()
        self._sweep_angle = snap.sweep_angle if snap.rev_points or snap.live_points else None

        world = [(p[1], p[2]) for p in cropped_rev] + [(p[1], p[2]) for p in cropped_live]
        pts_px = self.mapping.world_to_canvas(np.array(world) if world else np.zeros((0, 2)))
        bg_pts_px = self.mapping.world_to_canvas(np.array(bg_world) if bg_world else np.zeros((0, 2)))
        self.points_total = len(world)

        if self.args.debug and self.frame_count % 30 == 0:
            pr = f"x[{pts_px[:,0].min():.0f},{pts_px[:,0].max():.0f}] y[{pts_px[:,1].min():.0f},{pts_px[:,1].max():.0f}]" if len(pts_px) else "kosong"
            print(f"[debug] f={self.frame_count} fps={self.fps:.1f} raw={snap.points_per_rev} rev={len(cropped_rev)} live={len(cropped_live)} "
                  f"px={len(pts_px)} {pr} tracks={len(self.tracks)} "
                  f"rot={snap.rate_hz:.2f}Hz ok={snap.packets_ok} bad={snap.packets_bad} sweep={self._sweep_angle}")

        rings = []
        blobs = []
        if style.get("track_ring", True):
            for t in self.tracks:
                p = self.mapping.world_to_canvas(np.array([[t.x, t.y]]))[0]
                rings.append((float(p[0]), float(p[1]), float(style.get("track_ring_size", 44.0))))
                ppm = self._px_per_m()
                blobs.append((float(p[0]), float(p[1]), float(max(14.0, t.size * float(cfg["mask"].get("blob_scale", 3.0)) * ppm))))

        fan_origin, fan_arc = self._fan()
        zone = self._zone_poly() if style.get("zone_outline", True) else None
        grid = self._grid_segments()

        bg = tuple(cfg["canvas"].get("background", [0, 0, 0, 1])) if cfg["canvas"].get("opaque_background") else (0.0, 0.0, 0.0, 0.0)

        fbo = self.scene.render(
            pts_px=pts_px,
            color=style["point_color"],
            point_size=float(style.get("point_size", 5.0)),
            point_alpha=float(style.get("point_alpha", 0.9)),
            trail_enabled=bool(style.get("trail_enabled", True)),
            trail_decay=float(style.get("trail_decay", 0.87)),
            trail_intensity=float(style.get("trail_intensity", 1.0)),
            grid=grid,
            grid_width=2.0,
            grid_color=(*style.get("grid_color", [0.1, 0.35, 0.45]), float(style.get("grid_alpha", 0.3))),
            fan_origin=fan_origin,
            fan_arc=fan_arc,
            fan_color=tuple(style.get("sweep_color", [0.15, 0.75, 1.0])),
            fan_alpha=float(style.get("sweep_alpha", 0.1)),
            zone_poly=zone,
            zone_color=(*style.get("zone_color", [1.0, 0.55, 0.1]), 0.8),
            rings=rings,
            ring_color=(*style.get("track_ring_color", [1, 1, 1]), 0.55),
            bg_pts_px=bg_pts_px,
            bg_color=tuple(style.get("bg_point_color", [0.2, 0.6, 0.8])),
            bg_alpha=float(style.get("bg_point_alpha", 0.12)),
            bg_size=float(style.get("bg_point_size", 5.0)),
            lines=self._overlay_lines(),
            background=bg,
        )
        self.spout_beauty.send(fbo)

        mask_cfg = cfg["mask"]
        if mask_cfg.get("enabled", True):
            now_mask = time.perf_counter()
            mask_fps = float(cfg["spout"].get("mask_fps", 30.0))
            if now_mask - getattr(self, "_mask_sent_at", 0.0) >= 1.0 / max(1.0, mask_fps):
                self._mask_sent_at = now_mask
                mask_fbo = self.mask.render(
                    blobs=blobs,
                    points_px=pts_px,
                    color=tuple(mask_cfg.get("color", [1, 1, 1])),
                    intensity=float(mask_cfg.get("intensity", 1.0)),
                    decay=float(mask_cfg.get("decay", 0.0)),
                    invert=bool(mask_cfg.get("invert", False)),
                    source=str(mask_cfg.get("source", "tracks")),
                )
                self.spout_mask.send(mask_fbo)

    # ------------------------------------------------------------------ main loop
    def run(self) -> None:
        target_dt = 1.0 / self.fps_target
        last = time.perf_counter()
        fps_ema = 0.0
        snapshot_at = int(self.args.frames) if self.args.frames else 0

        while self.running and not glfw.window_should_close(self.win):
            glfw.poll_events()
            self.impl.process_inputs()
            self._handle_keys()

            now = time.perf_counter()
            dt = now - last
            last = now
            if dt > 0:
                fps_ema = lerp(fps_ema, 1.0 / dt, 0.08)
            self.fps = fps_ema

            snap = self.source.snapshot()

            fb_w, fb_h = glfw.get_framebuffer_size(self.win)
            if self.show_preview:
                self._render_frame(snap)
                self.ctx.screen.use()
                self.ctx.viewport = (0, 0, fb_w, fb_h)
                self.ctx.clear(0.02, 0.02, 0.025, 1.0)
                rx, ry, rw, rh = self._preview_rect(fb_w, fb_h)
                self.scene.draw_preview((rx, fb_h - ry - rh, rw, rh), flip_y=True)
                if self.edit_quad:
                    self._handle_quad_drag(fb_w, fb_h)
                else:
                    self._handle_preview_input(fb_w, fb_h)
            else:
                self._render_frame(snap)

            self._publish_osc()
            self._publish_tuio()

            if not self.args.headless and (self.show_panel or self.show_preview):
                self.ctx.screen.use()
                self.ctx.viewport = (0, 0, fb_w, fb_h)
                imgui.new_frame()
                if self.show_preview:
                    self._draw_area_overlay()
                if self.show_panel:
                    actions = draw_panel(self.cfg, self._panel_status(snap))
                    self._apply_actions(actions)
                imgui.render()
                self.impl.render(imgui.get_draw_data())

            glfw.swap_buffers(self.win)
            self.frame_count += 1

            if snapshot_at and self.frame_count >= snapshot_at:
                if self.args.snapshot:
                    self.save_snapshot(self.args.snapshot)
                break

            remaining = target_dt - (time.perf_counter() - now)
            if remaining > 0:
                time.sleep(remaining)

        self.shutdown()

    def _panel_status(self, snap) -> dict:
        return {
            "connected": snap.connected,
            "fps": self.fps,
            "rate_hz": snap.rate_hz,
            "motor_hz": snap.motor_hz,
            "points_per_rev": snap.points_per_rev,
            "points_total": self.points_total,
            "packets_ok": snap.packets_ok,
            "packets_bad": snap.packets_bad,
            "error": snap.error or self.spout_beauty.error or self.osc.error,
            "spout_beauty": "" if self.spout_beauty.error else self.spout_beauty.name,
            "spout_mask": "" if self.spout_mask.error else self.spout_mask.name,
            "edit_quad": self.edit_quad,
            "bg_ratio": self.background.learned_ratio,
            "canvas_w": self.canvas_w,
            "canvas_h": self.canvas_h,
            "px_per_m": self.mapping.px_per_m(),
            "stage_mode": self.mapping.is_stage,
            "area_store": self.area_store,
            "area_editor": self.area_editor,
            "area_metrics": self.area_metrics,
            "tuio_status": (self.tuio.error or
                            (f"TUIO aktif · {self.tuio.frames_sent} frame terkirim"
                             if self.tuio.enabled else "TUIO nonaktif")),
        }

    def _apply_actions(self, actions: dict) -> None:
        if actions.get("save"):
            save_config(self.cfg_path, self.cfg)
            print(f"[config] disimpan ke {self.cfg_path}")
        if actions.get("reconnect"):
            self._start_source()
        if actions.get("reset"):
            self.scene.trail.begin(0.0)
            self.mask.ping.begin(0.0)
        if actions.get("learn"):
            self.background.reset()
            self._learn_until = time.perf_counter() + float(self.cfg["filter"].get("learn_seconds", 4.0))
            self.background.learning = True
        if actions.get("reset_quad"):
            self._mapper_sig = None
        if actions.get("quit"):
            self.running = False
        if actions.get("edit_quad") is not None and "edit_quad" in actions:
            self.edit_quad = bool(actions["edit_quad"])

    def save_snapshot(self, path: str) -> None:
        self._save_fbo(self.scene.out_fbo, path)
        base = Path(path)
        mask_path = base.with_name(base.stem + "_mask" + base.suffix)
        if self.cfg["mask"].get("enabled", True):
            self._save_fbo(self.mask.out_fbo, str(mask_path))

    def _save_fbo(self, fbo, path: str) -> None:
        data = fbo.read(components=4, dtype="f1")
        arr = np.frombuffer(data, dtype=np.uint8).reshape(self.canvas_h, self.canvas_w, 4)
        arr = np.flipud(arr)
        try:
            from PIL import Image
            Image.fromarray(arr, "RGBA").save(path)
            print(f"[snapshot] {path}")
        except Exception as exc:
            print(f"[snapshot] gagal simpan: {exc}")

    def shutdown(self) -> None:
        try:
            self.source.stop()
        except Exception:
            pass
        for sp in (self.spout_beauty, self.spout_mask):
            try:
                sp.release()
            except Exception:
                pass
        self.osc.close()
        try:
            self.tuio.close()
        except Exception:
            pass
        try:
            self.impl.shutdown()
        except Exception:
            pass
        glfw.terminate()


def default_config_path() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent / "config.json"
    return Path(__file__).resolve().parent.parent / "config.json"


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Lidar To Spout — YDLIDAR X3 scanner visual untuk Spout/Resolume")
    p.add_argument("--config", default=str(default_config_path()))
    p.add_argument("--port", default="")
    p.add_argument("--baud", type=int, default=0)
    p.add_argument("--replay", default="", help="file .bin hasil rekaman")
    p.add_argument("--speed", type=float, default=0.0, help="kecepatan replay (x)")
    p.add_argument("--width", type=int, default=0)
    p.add_argument("--height", type=int, default=0)
    p.add_argument("--fps", type=int, default=0)
    p.add_argument("--headless", action="store_true", help="tanpa window (Spout tetap jalan)")
    p.add_argument("--no-spout", action="store_true")
    p.add_argument("--no-osc", action="store_true")
    p.add_argument("--no-tuio", action="store_true")
    p.add_argument("--no-trail", action="store_true")
    p.add_argument("--snapshot", default="", help="simpan satu frame PNG lalu keluar")
    p.add_argument("--frames", type=int, default=0, help="jumlah frame sebelum snapshot")
    p.add_argument("--debug", action="store_true")
    args = p.parse_args(argv)
    if args.snapshot and not args.frames:
        args.frames = 120
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    app = App(Path(args.config), args)
    try:
        app.run()
    except KeyboardInterrupt:
        app.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
