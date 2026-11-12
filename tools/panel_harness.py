"""Harness panel imgui: render semua section dengan berbagai state (tanpa interaksi).

Menangkap error signature/atribut di panel yang jarang dibuka.
    python tools/panel_harness.py
"""
from __future__ import annotations

import copy
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

import glfw  # noqa: E402
import imgui  # noqa: E402
import moderngl  # noqa: E402
from imgui.integrations.glfw import GlfwRenderer  # noqa: E402

from config import DEFAULTS  # noqa: E402
from ui.area_editor import AreaEditor  # noqa: E402
from ui.areas import AreaStore  # noqa: E402
from ui.panel import draw_panel  # noqa: E402


def make_status(store, editor, metrics, edit_quad=False):
    return {
        "connected": True, "fps": 59.8, "rate_hz": 10.9, "motor_hz": 10.8,
        "points_per_rev": 230, "points_total": 460, "packets_ok": 1234,
        "packets_bad": 0, "error": "", "spout_beauty": "Lidar Beauty",
        "spout_mask": "Lidar Mask", "edit_quad": edit_quad,
        "bg_ratio": 0.62, "canvas_w": 1920, "canvas_h": 1080,
        "px_per_m": (240.0, 270.0), "stage_mode": True,
        "area_store": store, "area_editor": editor, "area_metrics": metrics,
        "tuio_status": "TUIO aktif · 0 frame terkirim",
    }


def main() -> int:
    if not glfw.init():
        print("glfw gagal")
        return 1
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    win = glfw.create_window(1280, 720, "panel", None, None)
    if not win:
        print("window gagal")
        return 1
    glfw.make_context_current(win)
    moderngl.create_context()
    imgui.create_context()
    impl = GlfwRenderer(win)
    imgui.get_io().ini_file_name = None

    cfg = copy.deepcopy(DEFAULTS)
    cfg["ui"]["preview_height"] = 900
    store = AreaStore(cfg)
    editor = AreaEditor(store)
    store.add_rect((1, 1), (3, 2))
    store.add_poly([(5, 0), (7, 0), (7, 2), (5, 2)])
    store.items[1].overlap = True
    store.items[1].enabled = False
    cfg["areas"]["output_area"] = store.items[0].id
    cfg["stage"]["canvas"]["aspect"] = "custom"
    cfg["stage"]["resolution"]["mode"] = "fixed"
    cfg["mask"]["source"] = "points"
    cfg["map"]["quad"] = [[0, 0], [1, 0], [1, 1], [0, 1]]

    metrics = {a.id: {"presence": 1, "count": 2, "cx": 2.0, "cy": 1.5,
                      "u": 0.3, "v": 0.4, "speed": 0.8} for a in store.items}

    states = [
        ("idle", editor),
        ("busy-poly", editor),
        ("busy-rect", editor),
        ("unlocked", editor),
    ]

    ok = True
    frame = 0
    for name, ed in states:
        if name == "busy-poly":
            ed.arm_poly()
            ed.on_down((0.5, 0.5))
            ed.on_down((1.5, 1.5))
        elif name == "busy-rect":
            ed.cancel()
            ed.arm_rect()
            ed.on_down((0.5, 0.5))
        elif name == "unlocked":
            ed.cancel()
            ed.unlocked = True
        for _ in range(4):
            glfw.poll_events()
            impl.process_inputs()
            imgui.new_frame()
            try:
                draw_panel(cfg, make_status(store, ed, metrics, edit_quad=(frame % 2 == 0)))
            except Exception as exc:  # pragma: no cover
                print(f"FAIL state={name} frame={frame}: {type(exc).__name__}: {exc}")
                ok = False
                break
            imgui.render()
            impl.render(imgui.get_draw_data())
            frame += 1
            time.sleep(0.01)
        print(f"  {'PASS' if ok else 'FAIL'}  state={name}")

    impl.shutdown()
    glfw.terminate()
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
