"""Harness input viewport: simulasi klik/drag untuk menguji editor area & marker.

Memakai App asli (proses GL tersembunyi) dengan glfw dimock.
    python tools/input_harness.py
"""
from __future__ import annotations

import copy
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

import glfw  # noqa: E402
import numpy as np  # noqa: E402

from app import App, parse_args  # noqa: E402
from config import DEFAULTS  # noqa: E402

WIN = (1280, 420)
PASS = 0
FAIL = 0
STATE = {"down": False, "pos": (0.0, 0.0)}


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name}")
    else:
        FAIL += 1
        print(f"  FAIL  {name}  {detail}")


def patch_glfw():
    glfw.get_mouse_button = lambda win, btn: glfw.PRESS if STATE["down"] else glfw.RELEASE
    glfw.get_cursor_pos = lambda win: STATE["pos"]
    glfw.get_window_size = lambda win: WIN
    glfw.get_key = lambda win, key: glfw.RELEASE


def canvas_to_window(app, cx, cy):
    rx, ry, rw, rh = app._preview_rect_window()
    return (rx + cx / app.canvas_w * rw, ry + cy / app.canvas_h * rh)


def to_canvas(app, room_x, room_y):
    p = app.mapping.room_to_canvas([[room_x, room_y]])[0]
    return canvas_to_window(app, float(p[0]), float(p[1]))


def step(app, down, pos):
    STATE["down"] = down
    STATE["pos"] = pos
    app._handle_preview_input(WIN[0], WIN[1])


def main():
    patch_glfw()
    cfg = copy.deepcopy(DEFAULTS)
    cfg["serial"]["replay_file"] = "data/demo_room.bin"
    cfg["spout"]["enabled"] = False
    cfg["osc"]["enabled"] = False
    cfg["tuio"]["enabled"] = False
    tmp = Path(tempfile.gettempdir()) / "opencode" / "cfg_input.json"
    tmp.parent.mkdir(parents=True, exist_ok=True)
    tmp.write_text(json.dumps(cfg, indent=2), encoding="utf-8")

    args = parse_args(["--config", str(tmp), "--headless", "--no-spout", "--no-osc", "--no-tuio"])
    app = App(tmp, args)
    try:
        # 1. buat rectangle dengan drag di dalam preview
        app.area_editor.arm_rect()
        step(app, True, to_canvas(app, 1.0, 1.0))
        step(app, True, to_canvas(app, 3.0, 2.0))
        step(app, False, to_canvas(app, 3.0, 2.0))
        check("drag membuat rectangle", len(app.area_store.items) == 1,
              f"items={len(app.area_store.items)}")
        check("mode kembali idle", app.area_editor.mode == "idle")

        # 2. drag rectangle berikutnya tapi dilepas DI LUAR preview (letterbox)
        app.area_editor.arm_rect()
        step(app, True, to_canvas(app, 5.0, 1.0))
        step(app, True, to_canvas(app, 6.5, 2.0))
        step(app, False, (2.0, 2.0))  # pojok kiri-atas window = di luar gambar
        check("release di luar preview tidak stuck", app.area_editor.mode == "idle",
              f"mode={app.area_editor.mode}")

        # 3. geser area pertama (klik dalam, geser, lepas di luar)
        first = app.area_store.items[0]
        before = [list(p) for p in first.points]
        step(app, True, to_canvas(app, 2.0, 1.5))
        step(app, True, to_canvas(app, 2.4, 1.5))
        step(app, False, to_canvas(app, 2.4, 1.5))
        moved = abs(first.points[0][0] - before[0][0]) > 0.05
        check("drag area menggeser posisi", moved,
              f"before={before[0]} after={first.points[0]}")

        # 4. klik area kedua lalu drag vertex setelah unlock
        app.area_store.selected_id = app.area_store.items[0].id
        app.area_editor.unlocked = True
        area = app.area_store.selected
        vx, vy = area.points[0][0], area.points[0][1]
        p = app.mapping.room_to_canvas([[vx, vy]])[0]
        wp = canvas_to_window(app, float(p[0]), float(p[1]))
        step(app, True, wp)
        check("handle vertex terdeteksi", app.area_editor.mode == "drag_vertex",
              f"mode={app.area_editor.mode}")
        step(app, True, (wp[0] + 20, wp[1] + 10))
        step(app, False, (wp[0] + 20, wp[1] + 10))
        check("vertex berpindah", app.area_editor.mode == "idle" and area.points[0] != [vx, vy])

        # 5. drag marker LiDAR
        app.area_editor.unlocked = False
        lidar = app.cfg["stage"]["lidar"]
        old = (lidar["x_m"], lidar["y_m"])
        mp = app.mapping.room_to_canvas([[old[0], old[1]]])[0]
        mw = canvas_to_window(app, float(mp[0]), float(mp[1]))
        step(app, True, mw)
        step(app, True, (mw[0] + 30, mw[1] + 20))
        step(app, False, (mw[0] + 30, mw[1] + 20))
        moved_marker = abs(lidar["x_m"] - old[0]) > 0.01 or abs(lidar["y_m"] - old[1]) > 0.01
        check("marker LiDAR bisa digeser", moved_marker,
              f"old={old} new={(lidar['x_m'], lidar['y_m'])}")

        # 6. polygon via ctrl+klik sudah diuji di unit test; pastikan editor bersih
        check("tidak ada state menggantung", app.area_editor.mode == "idle"
              and app._drag_lidar == 0, f"mode={app.area_editor.mode} drag={app._drag_lidar}")

        # 7. area yang sekaligus jadi output crop: drag tidak boleh runaway
        import time
        target = app.area_store.items[0]
        app.cfg["areas"]["output_area"] = target.id
        app._sync_stage(time.perf_counter())
        app._sync_stage(time.perf_counter() + 1.0)
        check("crop aktif", getattr(app.mapping, "crop", None) is not None)
        before = [list(p) for p in target.points]
        cxp, cyp = target.centroid()
        pp = app.mapping.room_to_canvas([[cxp, cyp]])[0]
        wp = canvas_to_window(app, float(pp[0]), float(pp[1]))
        step(app, True, wp)
        app._sync_stage(time.perf_counter() + 2.0)
        step(app, True, (wp[0] + 40, wp[1]))
        app._sync_stage(time.perf_counter() + 3.0)
        step(app, False, (wp[0] + 40, wp[1]))
        app._sync_stage(time.perf_counter() + 4.0)
        moved_x = target.points[0][0] - before[0][0]
        check("drag area crop tidak runaway", 0.0 < moved_x < 0.6, f"moved={moved_x:.3f} m")
        check("crop ikut area setelah drag",
              getattr(app.mapping, "crop", None) is not None
              and abs(app.mapping.crop[0] - target.bounds()[0]) < 0.05,
              f"crop={getattr(app.mapping, 'crop', None)} bounds={target.bounds()}")

        # 8. sinkronisasi pengaturan live (tanpa restart)
        app.cfg["osc"]["port"] = 9010
        app._sync_outputs()
        check("port OSC berubah live", app.osc.addr[1] == 9010, str(app.osc.addr))
        app.cfg["tuio"]["port"] = 3340
        app._sync_outputs()
        check("port TUIO berubah live", app.tuio.addr[1] == 3340, str(app.tuio.addr))

        app.cfg["tracker"]["gate_m"] = 0.91
        app._sync_tracker_settings()
        check("parameter tracker live", abs(app.tracker.cfg.gate_m - 0.91) < 1e-9,
              str(app.tracker.cfg.gate_m))

        old_bg = app.background
        app.cfg["filter"]["bins"] = 180
        app._sync_filter()
        check("filter bins dibangun ulang", app.background is not old_bg and app.background.bins == 180)

        app.spout_beauty.set_enabled(False)
        check("spout disable melepas sender", app.spout_beauty.enabled is False)
        app.spout_beauty.set_enabled(True)
        check("spout enable kembali", app.spout_beauty.enabled is True)

        # 9. efek titik berkedip (raw point)
        app.cfg["style"].update({"blink": True, "blink_mode": "strobe", "blink_rate_hz": 12.0,
                                 "blink_depth": 1.0, "blink_density": 0.5})
        a1 = app._blink_alphas(200, 1.00)
        a3 = app._blink_alphas(200, 1.02)   # seed sama (int(now*12) = 12)
        a2 = app._blink_alphas(200, 1.09)   # seed beda (int(now*12) = 13)
        check("blink: panjang & rentang alpha",
              a1 is not None and len(a1) == 200 and float(a1.min()) >= 0.0 and float(a1.max()) <= 1.0)
        check("blink: stabil dalam satu tick", np.array_equal(a1, a3))
        check("blink: berubah di tick berikutnya", not np.array_equal(a1, a2))
        app.cfg["style"]["blink_mode"] = "twinkle"
        t1 = app._blink_alphas(50, 1.0)
        t2 = app._blink_alphas(50, 1.2)
        check("twinkle: alpha berubah halus", t1 is not None and not np.array_equal(t1, t2))
        app.cfg["style"]["blink"] = False
        check("blink nonaktif -> None", app._blink_alphas(10, 1.0) is None)
    finally:
        app.shutdown()

    print(f"\n{PASS} lulus, {FAIL} gagal")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
