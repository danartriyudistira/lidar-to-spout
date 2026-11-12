"""Tes end-to-end: memastikan sender Spout aplikasi terlihat & mengirim frame.

Jalankan aplikasi di proses terpisah, lalu:
    python tools/spout_receiver_test.py --name "Lidar Beauty" --seconds 3
"""
import argparse
import time

import numpy as np
import SpoutGL
import glfw
import moderngl


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="Lidar Beauty")
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()

    if not glfw.init():
        print("FAIL: glfw init")
        return 1
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    win = glfw.create_window(64, 64, "receiver", None, None)
    if not win:
        print("FAIL: window")
        return 1
    glfw.make_context_current(win)
    moderngl.create_context()

    recv = SpoutGL.SpoutReceiver()
    recv.setReceiverName(args.name)
    recv.createOpenGL()
    print(f"receiver '{args.name}' dibuat, GLDX ready: {recv.isGLDXReady()}")

    frames = 0
    lit = 0
    maxv = 0
    width = height = 0
    buf = None
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        if recv.receiveTexture():
            frames += 1
            width = recv.getSenderWidth()
            height = recv.getSenderHeight()
            need = width * height * 4
            if buf is None or len(buf) != need:
                buf = bytearray(need)
            if recv.receiveImage(buf, 6408, False, 0):
                arr = np.frombuffer(bytes(buf), dtype=np.uint8)
                if arr.max() > 8:
                    lit += 1
                maxv = max(maxv, int(arr.max()))
        else:
            time.sleep(0.005)

    elapsed = time.time() - t0
    print(f"frame diterima: {frames} ({frames / elapsed:.1f} fps), ada isi: {lit}, max pixel: {maxv}, "
          f"ukuran: {width}x{height}")
    recv.releaseReceiver()
    recv.closeOpenGL()
    glfw.terminate()
    ok = frames > 5 and lit > 0
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
