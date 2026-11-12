"""Output Spout (2 sender: beauty + mask)."""
from __future__ import annotations

import moderngl

try:
    import SpoutGL
except Exception:  # pragma: no cover
    SpoutGL = None


class SpoutOutput:
    def __init__(self, name: str, width: int, height: int, invert: bool = True, enabled: bool = True) -> None:
        self.name = name
        self.w = width
        self.h = height
        self.invert = invert
        self.enabled = enabled and SpoutGL is not None
        self._sender = None
        self._fails = 0
        self.error = ""
        if SpoutGL is None:
            self.error = "SpoutGL tidak terpasang (pip install SpoutGL)"
            self.enabled = False

    def _ensure(self) -> bool:
        if self._sender is None:
            self._sender = SpoutGL.SpoutSender()
            self._sender.setSenderName(self.name)
            ok = self._sender.createOpenGL()
            if not ok:
                self.error = f"Spout sender '{self.name}' gagal dibuat"
                self._sender = None
                return False
        return True

    def send(self, fbo: moderngl.Framebuffer) -> bool:
        if not self.enabled:
            return False
        try:
            if not self._ensure():
                return False
            ok = self._sender.sendFbo(fbo.glo, self.w, self.h, self.invert)
            if not ok:
                self._fails += 1
                if self._fails == 1:
                    self.error = f"Spout send gagal untuk '{self.name}'"
            return bool(ok)
        except Exception as exc:  # pragma: no cover
            self.error = f"Spout error: {exc}"
            return False

    def resize(self, width: int, height: int) -> None:
        if (width, height) != (self.w, self.h):
            self.w, self.h = width, height
            self.release()
            self._ensure()

    def set_enabled(self, value: bool) -> None:
        value = bool(value) and SpoutGL is not None
        if value == self.enabled:
            return
        self.enabled = value
        if value:
            self._ensure()
        else:
            self.release()

    def release(self) -> None:
        if self._sender is not None:
            try:
                self._sender.releaseSender()
                self._sender.closeOpenGL()
            except Exception:
                pass
            self._sender = None
