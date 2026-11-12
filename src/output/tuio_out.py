"""TUIO 1.1 /tuio/2Dcur sender (UDP), tanpa dependency eksternal."""
from __future__ import annotations

import socket
import struct

from .osc_out import osc_message

BUNDLE_HEADER = b"#bundle\x00"


def osc_bundle(messages: list[bytes]) -> bytes:
    out = bytearray(BUNDLE_HEADER)
    out += struct.pack(">Q", 0)  # timetag 0 = immediately
    for msg in messages:
        out += struct.pack(">i", len(msg))
        out += msg
    return bytes(out)


class TuioSender:
    """Kirim /tuio/2Dcur alive + set + fseq dalam satu bundle per frame."""

    def __init__(self, host: str = "127.0.0.1", port: int = 3333, enabled: bool = True) -> None:
        self.enabled = enabled
        self.addr = (host, int(port))
        self._sock: socket.socket | None = None
        self._seq = 0
        self.error = ""
        self.frames_sent = 0
        if enabled:
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            except Exception as exc:  # pragma: no cover
                self.error = f"TUIO socket gagal: {exc}"
                self.enabled = False

    def send(self, cursors: list[tuple[int, float, float, float, float]]) -> bool:
        """cursors: list (session_id, u, v, speed_norm, size_norm)."""
        if not self.enabled or self._sock is None:
            return False
        try:
            msgs = [osc_message("/tuio/2Dcur", "alive", *[int(c[0]) for c in cursors])]
            for sid, u, v, speed, _size in cursors:
                msgs.append(osc_message("/tuio/2Dcur", "set",
                                        int(sid), float(u), float(v),
                                        0.0, 0.0, float(speed)))
            self._seq = (self._seq + 1) % 2147483647
            msgs.append(osc_message("/tuio/2Dcur", "fseq", int(self._seq)))
            self._sock.sendto(osc_bundle(msgs), self.addr)
            self.frames_sent += 1
            return True
        except Exception as exc:  # pragma: no cover
            self.error = f"TUIO send error: {exc}"
            return False

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
