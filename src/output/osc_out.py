"""OSC sender minimal (UDP, tanpa dependency eksternal)."""
from __future__ import annotations

import socket
import struct


def _pad(data: bytes) -> bytes:
    """OSC string padding: minimal satu terminator null, total kelipatan 4."""
    return data + b"\x00" * (4 - len(data) % 4)


def osc_message(address: str, *args) -> bytes:
    tags = ","
    payload = b""
    for a in args:
        if isinstance(a, bool):
            tags += "T" if a else "F"
        elif isinstance(a, int):
            tags += "i"
            payload += struct.pack(">i", int(a))
        elif isinstance(a, float):
            tags += "f"
            payload += struct.pack(">f", float(a))
        elif isinstance(a, str):
            tags += "s"
            payload += _pad(a.encode("utf-8"))
        else:
            tags += "f"
            payload += struct.pack(">f", float(a))
    return _pad(address.encode("utf-8")) + _pad(tags.encode("utf-8")) + payload


class OscSender:
    def __init__(self, host: str = "127.0.0.1", port: int = 9000, enabled: bool = True) -> None:
        self.enabled = enabled
        self.addr = (host, int(port))
        self._sock: socket.socket | None = None
        self.error = ""
        if enabled:
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            except Exception as exc:  # pragma: no cover
                self.error = f"OSC socket gagal: {exc}"
                self.enabled = False

    def send(self, address: str, *args) -> None:
        if not self.enabled or self._sock is None:
            return
        try:
            self._sock.sendto(osc_message(address, *args), self.addr)
        except Exception as exc:  # pragma: no cover
            self.error = f"OSC send error: {exc}"

    def consume_errors(self) -> str:
        e = self.error
        self.error = ""
        return e

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
