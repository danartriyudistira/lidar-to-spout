"""Parser protokol YDLIDAR X3 (header AA 55, sample 2 byte, checksum XOR)."""
from __future__ import annotations

from dataclasses import dataclass
from math import atan, degrees

HEADER0 = 0xAA
HEADER1 = 0x55
CHECKSUM_SEED = 0x55AA
MIN_FRAME = 10
MAX_LSN = 100


@dataclass(slots=True)
class Packet:
    ct: int
    lsn: int
    fsa: int
    lsa: int
    samples: tuple[int, ...]

    @property
    def is_zero_packet(self) -> bool:
        return bool(self.ct & 0x01)

    @property
    def scan_freq_hz(self) -> float:
        """Scan frequency hanya valid pada zero packet (CT >> 1 = freq x10)."""
        return (self.ct >> 1) / 10.0 if self.is_zero_packet else 0.0


class PacketParser:
    """State machine parser: feed() byte chunk, keluar paket valid."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames_ok = 0
        self.frames_bad = 0
        self.resyncs = 0

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> list[Packet]:
        self._buf.extend(data)
        out: list[Packet] = []
        while True:
            pkt = self._next()
            if pkt is None:
                break
            out.append(pkt)
        return out

    def _next(self) -> Packet | None:
        buf = self._buf
        while True:
            if len(buf) < MIN_FRAME:
                return None
            if buf[0] != HEADER0 or buf[1] != HEADER1:
                idx = buf.find(bytes((HEADER0, HEADER1)), 1)
                if idx < 0:
                    # simpan satu byte terakhir kalau dia calon header
                    del buf[: len(buf) - (1 if buf[-1] == HEADER0 else 0)]
                    self.resyncs += 1
                    return None
                del buf[:idx]
                self.resyncs += 1
                if len(buf) < MIN_FRAME:
                    return None

            ct = buf[2]
            lsn = buf[3]
            if lsn == 0 or lsn > MAX_LSN:
                del buf[:1]
                self.frames_bad += 1
                continue

            total = MIN_FRAME + 2 * lsn
            if len(buf) < total:
                return None

            fsa = buf[4] | (buf[5] << 8)
            lsa = buf[6] | (buf[7] << 8)
            cs = buf[8] | (buf[9] << 8)

            chk = CHECKSUM_SEED ^ fsa ^ (lsn << 8 | ct) ^ lsa
            samples = []
            p = MIN_FRAME
            for _ in range(lsn):
                si = buf[p] | (buf[p + 1] << 8)
                chk ^= si
                samples.append(si)
                p += 2

            if chk != cs:
                del buf[:1]
                self.frames_bad += 1
                continue

            del buf[:total]
            self.frames_ok += 1
            return Packet(ct=ct, lsn=lsn, fsa=fsa, lsa=lsa, samples=tuple(samples))


def ang_correct_deg(dist_mm: float) -> float:
    """Koreksi sudut LiDAR triangulasi (Skalar)."""
    if dist_mm <= 0.0:
        return 0.0
    return degrees(atan(21.8 * (155.3 - dist_mm) / (155.3 * dist_mm)))


def packet_start_angle(pkt: Packet) -> float:
    """Sudut awal paket tanpa koreksi (dipakai untuk deteksi batas revolusi)."""
    return ((pkt.fsa >> 1) / 64.0) % 360.0


def packet_points(pkt: Packet, correct_angle: bool = True, min_dist_mm: float = 0.0) -> list[tuple[float, float]]:
    """Konversi paket -> list (angle_deg, dist_mm). Zero packet tidak menghasilkan titik."""
    if pkt.is_zero_packet or pkt.lsn == 0:
        return []

    a0 = (pkt.fsa >> 1) / 64.0
    a1 = (pkt.lsa >> 1) / 64.0
    diff = a1 - a0
    if diff < 0:
        diff += 360.0

    n = pkt.lsn
    out: list[tuple[float, float]] = []
    for i, si in enumerate(pkt.samples):
        d_mm = si / 4.0
        if d_mm <= min_dist_mm:
            continue
        ang = a0 + (diff * i / (n - 1) if n > 1 else 0.0)
        if correct_angle and d_mm > 0.0:
            ang += ang_correct_deg(d_mm)
        out.append((ang % 360.0, d_mm))
    return out
