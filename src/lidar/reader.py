"""Sumber data scan: serial live dan replay file .bin."""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from .protocol import PacketParser, packet_points, packet_start_angle
from .geometry import polar_to_xy


@dataclass
class ScanSnapshot:
    rev_points: list[tuple[float, float, float]] = field(default_factory=list)
    live_points: list[tuple[float, float, float]] = field(default_factory=list)
    sweep_angle: float = 0.0
    rev_index: int = 0
    rate_hz: float = 0.0
    motor_hz: float = 0.0
    packets_ok: int = 0
    packets_bad: int = 0
    points_per_rev: int = 0
    connected: bool = False
    error: str = ""
    updated_at: float = 0.0


class _SourceBase(threading.Thread):
    daemon = True

    def __init__(self) -> None:
        super().__init__(name="lidar-source")
        self._lock = threading.Lock()
        self._snap = ScanSnapshot()
        self._stop = threading.Event()
        self._parser = PacketParser()
        self._rev: list[tuple[float, float, float]] = []
        self._live: list[tuple[float, float, float]] = []
        self._rev_times: list[float] = []
        self._rev_index = 0
        self._rev_avg = 0.0
        self._prev_angle: float | None = None
        self._last_zero = 0.0
        self._motor_hz = 0.0
        self._last_data_at = 0.0

    # ---- API publik -------------------------------------------------
    def snapshot(self) -> ScanSnapshot:
        with self._lock:
            s = self._snap
            return ScanSnapshot(
                rev_points=list(s.rev_points),
                live_points=list(s.live_points),
                sweep_angle=s.sweep_angle,
                rev_index=s.rev_index,
                rate_hz=s.rate_hz,
                motor_hz=s.motor_hz,
                packets_ok=s.packets_ok,
                packets_bad=s.packets_bad,
                points_per_rev=s.points_per_rev,
                connected=s.connected,
                error=s.error,
                updated_at=s.updated_at,
            )

    def stop(self) -> None:
        self._stop.set()

    # ---- internal ---------------------------------------------------
    def _publish(self) -> None:
        now = time.monotonic()
        with self._lock:
            self._snap.live_points = list(self._live)
            self._snap.rev_index = self._rev_index
            self._snap.packets_ok = self._parser.frames_ok
            self._snap.packets_bad = self._parser.frames_bad
            self._snap.motor_hz = self._motor_hz
            self._snap.updated_at = now
            if len(self._rev_times) >= 2:
                span = self._rev_times[-1] - self._rev_times[0]
                self._snap.rate_hz = (len(self._rev_times) - 1) / span if span > 0 else 0.0
            self._snap.connected = (now - self._last_data_at) < 1.0

    def _consume(self, data: bytes) -> None:
        self._last_data_at = time.monotonic()
        for pkt in self._parser.feed(data):
            now = time.monotonic()
            if pkt.is_zero_packet:
                self._motor_hz = pkt.scan_freq_hz or self._motor_hz
                self._last_zero = now
                self._close_rev()
                continue
            pts = packet_points(pkt, min_dist_mm=60.0)
            if not pts:
                continue
            a_start = packet_start_angle(pkt)
            # fallback: kalau lidar tidak mengirim zero packet, pakai lompatan sudut
            if (self._prev_angle is not None and a_start < self._prev_angle - 180.0
                    and now - self._last_zero > 1.0):
                self._close_rev()
            self._prev_angle = a_start
            for ang, d_mm in pts:
                if d_mm <= 0.0:
                    continue
                x, y = polar_to_xy(ang, d_mm / 1000.0)
                self._rev.append((ang, x, y))
                self._live.append((ang, x, y))
            with self._lock:
                self._snap.sweep_angle = pts[-1][0]
        self._publish()

    def _close_rev(self) -> None:
        # buang revolusi yang terpotong (mis. saat replay loop / start di tengah scan)
        min_points = max(16, int(0.25 * self._rev_avg)) if self._rev_avg else 16
        if len(self._rev) < min_points:
            self._rev.clear()
            self._live.clear()
            return
        self._rev_avg = len(self._rev) if not self._rev_avg else 0.8 * self._rev_avg + 0.2 * len(self._rev)
        now = time.monotonic()
        self._rev_times.append(now)
        self._rev_times = self._rev_times[-16:]
        self._rev_index += 1
        with self._lock:
            self._snap.rev_points = self._rev
            self._snap.points_per_rev = len(self._rev)
            self._snap.live_points = []
        self._rev = []
        self._live = []


class SerialSource(_SourceBase):
    def __init__(self, port: str, baud: int = 115200, read_bytes: int = 4096) -> None:
        super().__init__()
        self.port = port
        self.baud = baud
        self.read_bytes = read_bytes

    def run(self) -> None:
        try:
            import serial  # pyserial
        except Exception as exc:  # pragma: no cover
            self._fail(f"pyserial tidak tersedia: {exc}")
            return

        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as exc:
            self._fail(
                f"Gagal membuka {self.port}: {exc}. "
                "Pastikan port tidak dipakai aplikasi lain (mis. LIDARLOMBOK_BARU.exe)."
            )
            return

        self._last_data_at = time.monotonic()
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        while not self._stop.is_set():
            try:
                data = ser.read(self.read_bytes)
            except Exception as exc:
                self._fail(f"Serial error: {exc}")
                break
            if data:
                self._consume(data)
            else:
                self._publish()
        try:
            ser.close()
        except Exception:
            pass

    def _fail(self, msg: str) -> None:
        with self._lock:
            self._snap.error = msg
            self._snap.connected = False
        print(f"[lidar] {msg}")


class ReplaySource(_SourceBase):
    """Memutar file .bin dengan timing mirip serial (untuk development tanpa hardware)."""

    def __init__(self, path: str | Path, baud: int = 115200, speed: float = 1.0, loop: bool = True) -> None:
        super().__init__()
        self.path = Path(path)
        self.baud = baud
        self.speed = max(0.05, speed)
        self.loop = loop

    def run(self) -> None:
        if not self.path.exists():
            self._fail_replay(f"File replay tidak ada: {self.path}")
            return
        data = self.path.read_bytes()
        if not data:
            self._fail_replay("File replay kosong.")
            return

        bytes_per_sec = self.baud / 10.0 * self.speed
        chunk = 512
        while not self._stop.is_set():
            t0 = time.perf_counter()
            pos = 0
            while pos < len(data) and not self._stop.is_set():
                self._consume(data[pos : pos + chunk])
                pos += chunk
                target = t0 + pos / bytes_per_sec
                sleep = target - time.perf_counter()
                if sleep > 0:
                    time.sleep(min(sleep, 0.2))
            if not self.loop:
                self._publish()
                return
        self._publish()

    def _fail_replay(self, msg: str) -> None:
        with self._lock:
            self._snap.error = msg
        print(f"[replay] {msg}")
