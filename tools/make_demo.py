"""Generator rekaman demo sintetis (tanpa hardware) untuk mode --replay.

Membuat stream protokol YDLIDAR X3 palsu: ruangan 8x4 m, dua "orang" berjalan,
plus beberapa furnitur statis. Hasilnya file .bin yang bisa diputar aplikasi.

    python tools/make_demo.py --out data/demo_room.bin --seconds 8
"""
from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path

HEADER = b"\xAA\x55"
CHECKSUM_SEED = 0x55AA


def ray_rect(ox, oy, dx, dy, x0, y0, x1, y1):
    tmin, tmax = -1e9, 1e9
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-9:
            if o < lo or o > hi:
                return None
        else:
            t1 = (lo - o) / d
            t2 = (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return None
    if tmax < 0:
        return None
    return tmin if tmin > 1e-4 else (tmax if tmax > 1e-4 else None)


def ray_circle(ox, oy, dx, dy, cx, cy, r):
    px, py = ox - cx, oy - cy
    b = px * dx + py * dy
    c = px * px + py * py - r * r
    disc = b * b - c
    if disc < 0:
        return None
    t = -b - math.sqrt(disc)
    return t if t > 1e-4 else None


def frame_packet(ct: int, samples: list[int], fsa: int, lsa: int) -> bytes:
    lsn = len(samples)
    chk = CHECKSUM_SEED ^ fsa ^ (lsn << 8 | ct) ^ lsa
    body = bytearray()
    for si in samples:
        chk ^= si
        body += struct.pack("<H", si)
    return (HEADER + bytes([ct & 0xFF, lsn & 0xFF])
            + struct.pack("<H", fsa) + struct.pack("<H", lsa)
            + struct.pack("<H", chk) + bytes(body))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/demo_room.bin")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--hz", type=float, default=11.0)
    ap.add_argument("--step-deg", type=float, default=0.6)
    args = ap.parse_args()

    room = (0.0, 0.0, 8.0, 4.0)
    lidar = (4.0, 2.0)
    desks = [(0.6, 0.5, 2.2, 1.4), (5.8, 2.6, 7.4, 3.5)]
    pillars = [(6.9, 0.4, 0.25), (1.2, 3.4, 0.22)]

    out = bytearray()
    revs = int(args.seconds * args.hz)
    for r in range(revs):
        t = r / args.hz
        people = [
            (4.0 + 1.6 * math.sin(2 * math.pi * t / 6.0), 2.0 + 0.9 * math.sin(2 * math.pi * t / 3.0), 0.18),
            (2.4 + 2.2 * math.cos(2 * math.pi * t / 7.0), 1.2 + 0.8 * math.cos(2 * math.pi * t / 4.0), 0.16),
        ]
        # zero packet sebagai penanda revolusi (sekaligus info frekuensi motor)
        out += frame_packet(0xC9, [0], 0x0000, 0x0000)

        angles = []
        a = 0.0
        while a < 360.0:
            angles.append(a)
            a += args.step_deg

        samples = []
        for ang in angles:
            rad = math.radians(ang)
            dx, dy = math.cos(rad), math.sin(rad)
            best = ray_rect(lidar[0], lidar[1], dx, dy, *room)
            for (x0, y0, x1, y1) in desks:
                hit = ray_rect(lidar[0], lidar[1], dx, dy, x0, y0, x1, y1)
                if hit is not None and (best is None or hit < best):
                    best = hit
            for (px, py, pr) in pillars:
                hit = ray_circle(lidar[0], lidar[1], dx, dy, px, py, pr)
                if hit is not None and (best is None or hit < best):
                    best = hit
            for (px, py, pr) in people:
                hit = ray_circle(lidar[0], lidar[1], dx, dy, px, py, pr)
                if hit is not None and (best is None or hit < best):
                    best = hit
            if best is None or best > 8.0:
                samples.append(0)
            else:
                samples.append(int(round(best * 1000.0 * 4.0)))  # Si = distance_mm * 4

        # pecah jadi paket maksimum 40 sampel
        i = 0
        while i < len(samples):
            chunk = samples[i:i + 40]
            n = len(chunk)
            a0 = angles[i]
            a1 = angles[i + n - 1] if n > 1 else a0
            fsa = int(round(a0 * 128)) & 0xFFFF
            lsa = int(round(a1 * 128)) & 0xFFFF
            out += frame_packet(0x00, chunk, fsa, lsa)
            i += n

    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(bytes(out))
    print(f"[demo] {path} ({len(out)} bytes, {revs} revolusi, {args.seconds:.1f} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
