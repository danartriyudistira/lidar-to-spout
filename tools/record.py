"""Rekam data serial LiDAR ke .bin + statistik. Berguna untuk development tanpa hardware.

Contoh:
    python tools/record.py --port COM4 --seconds 10 --out data/scan.bin
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from lidar.protocol import PacketParser, packet_points, packet_start_angle  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    try:
        import serial
    except Exception:
        print("pyserial tidak ada: pip install pyserial")
        return 1

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    parser = PacketParser()
    raw = bytearray()
    rev_pts: list = []
    revs: list[int] = []
    prev = None
    zero_packets = 0

    t0 = time.time()
    print(f"merekam {args.port} @ {args.baud} selama {args.seconds:.1f} s ...")
    try:
        while time.time() - t0 < args.seconds:
            data = ser.read(4096)
            if not data:
                continue
            raw.extend(data)
            for pkt in parser.feed(data):
                if pkt.is_zero_packet:
                    zero_packets += 1
                    continue
                pts = packet_points(pkt, min_dist_mm=60.0)
                if not pts:
                    continue
                a = packet_start_angle(pkt)
                if prev is not None and a < prev - 180.0:
                    revs.append(len(rev_pts))
                    rev_pts = []
                prev = a
                rev_pts.extend(pts)
    finally:
        ser.close()

    if rev_pts:
        revs.append(len(rev_pts))

    elapsed = time.time() - t0
    packets = parser.frames_ok
    print(f"bytes: {len(raw)}  paket ok: {packets} ({packets / elapsed:.0f}/s)  rusak: {parser.frames_bad}")
    print(f"revolusi: {len(revs)} ({len(revs) / elapsed:.2f} Hz)  titik total: {sum(revs)} "
          f"({sum(revs) / elapsed:.0f}/s)")
    if revs:
        print(f"titik/revolusi: min {min(revs)}  median {statistics.median(revs):.0f}  max {max(revs)}")
    print(f"zero packet: {zero_packets}")

    if args.out:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(bytes(raw))
        print(f"disimpan: {out} ({len(raw)} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
