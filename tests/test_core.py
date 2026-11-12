"""Uji inti tanpa hardware: protokol, stage, area, tracker, background, OSC/TUIO.

Jalankan:  python tests/test_core.py
"""
from __future__ import annotations

import copy
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from config import DEFAULTS  # noqa: E402
from lidar.background import BackgroundModel  # noqa: E402
from lidar.protocol import (CHECKSUM_SEED, HEADER0, HEADER1, PacketParser,  # noqa: E402
                            packet_points)
from lidar.stage import compute_canvas_size, make_mapper  # noqa: E402
from lidar.tracker import FootTracker, TrackerSettings  # noqa: E402
from output.osc_out import osc_message  # noqa: E402
from output.tuio_out import osc_bundle  # noqa: E402
from ui.area_editor import AreaEditor  # noqa: E402
from ui.areas import Area, AreaStore, compute_area_metrics  # noqa: E402

PASS = 0
FAIL = 0


def check(name: str, cond: bool, detail: str = "") -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name}")
    else:
        FAIL += 1
        print(f"  FAIL  {name}  {detail}")


def build_packet(ct: int, samples: list[int], fsa: int, lsa: int) -> bytes:
    lsn = len(samples)
    chk = CHECKSUM_SEED ^ fsa ^ (lsn << 8 | ct) ^ lsa
    body = bytearray()
    for si in samples:
        chk ^= si
        body += struct.pack("<H", si)
    return (bytes([HEADER0, HEADER1, ct, lsn]) + struct.pack("<H", fsa)
            + struct.pack("<H", lsa) + struct.pack("<H", chk) + bytes(body))


def test_protocol() -> None:
    print("protokol X3")
    p = PacketParser()
    pkts = p.feed(build_packet(0, [400, 800, 1200], 0, 128 * 2))
    check("1 paket valid", len(pkts) == 1 and p.frames_ok == 1 and p.frames_bad == 0)
    pts = packet_points(pkts[0])
    check("3 titik, jarak mm", len(pts) == 3 and abs(pts[1][1] - 200.0) < 1e-6)

    bad = bytearray(build_packet(0, [400], 0, 128))
    bad[9] ^= 0xFF
    p2 = PacketParser()
    p2.feed(bytes(bad) + build_packet(0, [400], 0, 128))
    check("checksum rusak dilewati", p2.frames_bad == 1 and p2.frames_ok == 1,
          f"ok={p2.frames_ok} bad={p2.frames_bad}")

    p3 = PacketParser()
    pkts3 = p3.feed(b"\x00\x01\x02" + build_packet(0xC9, [0], 0, 0))
    check("zero packet + resync", len(pkts3) == 1 and pkts3[0].is_zero_packet
          and abs(pkts3[0].scan_freq_hz - 10.0) < 1e-9)


def test_stage() -> None:
    print("stage / kanvas")
    cfg = copy.deepcopy(DEFAULTS)
    check("aspek 16:9 -> 1920x1080", compute_canvas_size(cfg) == (1920, 1080))
    cfg32 = copy.deepcopy(cfg)
    cfg32["stage"]["canvas"]["aspect"] = "32:9"
    check("aspek 32:9 -> 3840x1080", compute_canvas_size(cfg32) == (3840, 1080))
    cfgauto = copy.deepcopy(cfg)
    cfgauto["stage"]["canvas"]["aspect"] = "auto"
    check("aspek auto ruang 8x4 -> 2160x1080", compute_canvas_size(cfgauto) == (2160, 1080))

    m = make_mapper(cfg, 1920, 1080)
    corners = m.room_to_canvas([[0, 0], [8, 4]])
    check("ruang penuh ke kanvas", abs(corners[0][0]) < 1e-6 and abs(corners[1][0] - 1920) < 1e-6)

    mc = make_mapper(cfg, 1920, 1080, crop_rect=(2, 1, 6, 3))
    c0 = mc.room_to_canvas([[2, 1]])[0]
    c1 = mc.room_to_canvas([[6, 3]])[0]
    check("crop area -> kanvas", abs(c0[0]) < 1e-6 and abs(c1[1] - 1080) < 1e-6)

    pt = m.world_to_canvas([[1.0, 0.0]])[0]
    check("lidar center + 1m ke kanan", abs(pt[0] - (960 + 240)) < 1e-6)


def test_areas() -> None:
    print("area")
    cfg = copy.deepcopy(DEFAULTS)
    store = AreaStore(cfg)
    ed = AreaEditor(store)

    ed.arm_rect()
    ed.on_down((1, 1))
    ed.on_up((3, 2))
    check("buat rectangle", len(store.items) == 1 and store.items[0].type == "rect")

    ed.arm_poly()
    for p in [(5, 0), (7, 0), (7, 2), (5, 2)]:
        ed.on_down(p)
    ed.on_key("enter")
    check("buat polygon", len(store.items) == 2 and store.items[1].contains(6, 1))

    a = store.items[0]
    check("point-in-polygon", a.contains(2, 1.5) and not a.contains(3.5, 1.5))
    check("slug", a.slug == "area_1")

    store.save()
    store2 = AreaStore(copy.deepcopy(cfg))
    check("roundtrip config", [x.to_dict() for x in store.items] == [x.to_dict() for x in store2.items])

    # warna parsial tidak boleh crash
    bad = Area.from_dict({"id": 9, "name": "x", "type": "rect", "points": [[0, 0], [1, 0], [1, 1]], "color": [1.0]})
    check("warna parsial aman", len(bad.color) == 3)

    metrics = compute_area_metrics([a], [type("T", (), {"id": 3, "x": 2.0, "y": 1.5, "vx": 1.0, "vy": 0.0})()],
                                   lambda x, y: (x / 8.0, y / 4.0))
    check("metrik area", metrics[a.id]["count"] == 1 and abs(metrics[a.id]["u"] - 0.25) < 1e-9)


def test_tracker() -> None:
    print("tracker")
    tr = FootTracker(TrackerSettings(enter_revs=2, exit_revs=3, cluster_gap_m=0.3))
    pts = [(10.0, 5.0, 1.0), (10.4, 5.1, 1.0)]
    out = tr.update(pts, 0.1)
    check("track baru belum confirmed", len(out) == 0)
    check("track baru misses=0", all(t.misses == 0 for t in tr.tracks),
          f"misses={[t.misses for t in tr.tracks]}")
    out = tr.update(pts, 0.1)
    check("confirmed setelah 2 revolusi", len(out) == 1 and out[0].hits >= 2)

    for _ in range(4):
        tr.update([], 0.1)
    check("track hilang setelah exit", len(tr.tracks) == 0)


def test_background() -> None:
    print("background filter")
    bg = BackgroundModel(bins=360, tol_m=0.15, learn_revs=2)
    static = [(float(a), 3.0 * __import__("math").cos(__import__("math").radians(a)),
               3.0 * __import__("math").sin(__import__("math").radians(a))) for a in range(0, 360, 5)]
    for _ in range(4):
        bg.update(static)
    check("background terpelajar", bg.learned_ratio >= 0.15, f"ratio={bg.learned_ratio:.2f}")
    check("titik statis dibuang", len(bg.filter(static)) == 0, f"sisa={len(bg.filter(static))}")

    closer = [(0.0, 2.0, 0.0)]
    check("objek lebih dekat dipertahankan", len(bg.filter(closer)) == 1)


def test_osc_tuio() -> None:
    print("OSC / TUIO")
    msg = osc_message("/zone/presence", 1)
    check("OSC kelipatan 4 + terminator", len(msg) % 4 == 0 and msg[14:16] == b"\x00\x00")
    msg2 = osc_message("/foot/3/size", 1.5)
    check("OSC string 12 byte aman", len(msg2) == 24 and msg2[12:16] == b"\x00\x00\x00\x00")

    bundle = osc_bundle([msg, msg2])
    check("bundle header", bundle.startswith(b"#bundle\x00") and len(bundle) > len(msg) + len(msg2))


def test_config_migration() -> None:
    print("migrasi config lama")
    from config import _merge
    old = {
        "serial": {"port": "COM7"},
        "canvas": {"width": 3840, "height": 1080},
        "map": {"quad": [[0, 0], [3840, 0], [3840, 1080], [0, 1080]]},
        "areas": {"items": [{"id": 1, "name": "rusak", "points": [[0, 0]]}]},
    }
    cfg = _merge(DEFAULTS, old)
    check("key baru ditambahkan", all(k in cfg for k in ("stage", "areas", "tuio", "filter")))
    check("nilai lama dipertahankan", cfg["serial"]["port"] == "COM7"
          and cfg["canvas"]["width"] == 3840)
    check("default stage terisi", cfg["stage"]["room"]["width_m"] == 8.0)

    store = AreaStore(cfg)
    check("area rusak (<3 titik) dibuang", len(store.items) == 0)

    m = make_mapper(cfg, 1920, 1080)  # quad legacy dalam piksel
    c = m.room_to_canvas([[0, 0], [8, 4]])
    check("quad legacy piksel diskalakan", abs(c[0][0]) < 1e-6 and abs(c[1][0] - 1920) < 1e-6,
          f"{c}")


def main() -> int:
    test_protocol()
    test_stage()
    test_areas()
    test_tracker()
    test_background()
    test_osc_tuio()
    test_config_migration()
    print(f"\n{PASS} lulus, {FAIL} gagal")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
