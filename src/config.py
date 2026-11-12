"""Konfigurasi aplikasi (JSON) + default."""
from __future__ import annotations

import copy
import json
from pathlib import Path

DEFAULTS: dict = {
    "serial": {"port": "COM4", "baud": 115200, "replay_file": "", "replay_speed": 1.0},
    "canvas": {"width": 3840, "height": 1080, "fps": 60, "opaque_background": False,
               "background": [0.0, 0.0, 0.0, 1.0]},
    "crop": {"use_angle": True, "angle_min": 0.0, "angle_max": 360.0,
             "dist_min": 0.30, "dist_max": 5.50},
    "areas": {"enabled": True, "items": [], "show_in_output": False, "output_area": 0},
    "stage": {
        "enabled": True,
        "room": {"width_m": 8.0, "height_m": 4.0},
        "lidar": {"x_m": 4.0, "y_m": 2.0, "heading_deg": 0.0, "mirror_y": False},
        "canvas": {"aspect": "16:9", "aspect_custom": 2.0, "fit": "stretch"},
        "resolution": {"mode": "auto_height", "height": 1080, "width": 1920, "max_width": 7680},
    },
    "map": {"angle_offset_deg": 0.0, "mirror_y": False,
            "world_rect": [-5.0, -1.4, 5.0, 1.4],
            "quad": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]},
    "filter": {
        "enabled": True,
        "bins": 720,
        "tol_m": 0.12,
        "learn_revs": 3,
        "learn_seconds": 4.0,
        "auto_learn": True,
        "show_all": False,
        "show_background": True,
    },
    "style": {
        "point_color": [0.35, 0.95, 1.0],
        "point_size": 8.0,
        "point_alpha": 0.9,
        "point_style": "glow",
        "blink": False,
        "blink_mode": "strobe",
        "blink_rate_hz": 12.0,
        "blink_depth": 0.9,
        "blink_density": 0.55,
        "trail_enabled": True,
        "trail_decay": 0.87,
        "trail_intensity": 1.0,
        "sweep_enabled": True,
        "sweep_width_deg": 55.0,
        "sweep_color": [0.15, 0.75, 1.0],
        "sweep_alpha": 0.10,
        "grid_enabled": True,
        "grid_step_m": 1.0,
        "grid_color": [0.10, 0.35, 0.45],
        "grid_alpha": 0.30,
        "zone_outline": True,
        "zone_color": [1.0, 0.55, 0.10],
        "track_ring": True,
        "track_ring_size": 44.0,
        "track_ring_color": [1.0, 1.0, 1.0],
        "bg_point_color": [0.25, 0.65, 0.85],
        "bg_point_alpha": 0.25,
        "bg_point_size": 6.0,
        "room_outline": True,
        "room_color": [0.35, 0.55, 0.65],
        "lidar_marker": True,
        "marker_color": [1.0, 0.8, 0.25],
    },
    "tracker": {
        "enabled": True,
        "cluster_gap_m": 0.18,
        "min_points": 2,
        "max_extent_m": 0.8,
        "gate_m": 0.55,
        "smooth_pos": 0.55,
        "smooth_vel": 0.35,
        "enter_revs": 2,
        "exit_revs": 4,
        "max_tracks": 24,
    },
    "mask": {
        "enabled": True,
        "color": [1.0, 1.0, 1.0],
        "intensity": 1.0,
        "blob_scale": 3.0,
        "decay": 0.0,
        "invert": False,
        "source": "tracks",
    },
    "spout": {"enabled": True, "beauty": "Lidar Beauty", "mask": "Lidar Mask", "mask_fps": 30.0},
    "osc": {"enabled": True, "host": "127.0.0.1", "port": 9000, "rate_hz": 30},
    "tuio": {"enabled": True, "host": "127.0.0.1", "port": 3333, "rate_hz": 30, "area_filter": ""},
    "ui": {"preview_width": 1280, "preview_height": 420, "show_panel": True},
}


def _merge(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path: str | Path) -> dict:
    p = Path(path)
    if not p.exists():
        save_config(p, DEFAULTS)
        return copy.deepcopy(DEFAULTS)
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"[config] gagal baca {p}: {exc} -> pakai default")
        return copy.deepcopy(DEFAULTS)
    return _merge(DEFAULTS, data)


def save_config(path: str | Path, cfg: dict) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(cfg, indent=2), encoding="utf-8")
