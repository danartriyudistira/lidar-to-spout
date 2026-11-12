"""Ambil satu frame dari sender Spout dan simpan sebagai PNG (untuk cek orientasi).

    python tools/spout_capture.py --name "Lidar Beauty" --out out.png
"""
from __future__ import annotations

import argparse
import time

import glfw
import moderngl
import numpy as np
import SpoutGL


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="Lidar Beauty")
    ap.add_argument("--out", default="spout_capture.png")
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--delay", type=float, default=1.5, help="tunggu sebelum menangkap frame terakhir")
    ap.add_argument("--flatten", action="store_true", help="komposit alpha di atas hitam")
    args = ap.parse_args()

    if not glfw.init():
        print("glfw gagal")
        return 1
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    win = glfw.create_window(64, 64, "capture", None, None)
    if not win:
        print("window gagal")
        return 1
    glfw.make_context_current(win)
    moderngl.create_context()

    recv = SpoutGL.SpoutReceiver()
    recv.setReceiverName(args.name)
    recv.createOpenGL()

    buf = None
    w = h = 0
    grabbed_at = None
    t0 = time.time()
    while time.time() - t0 < args.timeout:
        if recv.receiveTexture():
            w = recv.getSenderWidth()
            h = recv.getSenderHeight()
            need = w * h * 4
            if buf is None or len(buf) != need:
                buf = bytearray(need)
            if recv.receiveImage(buf, 6408, False, 0):
                if grabbed_at is None:
                    grabbed_at = time.time()
                elif time.time() - grabbed_at >= args.delay:
                    arr = np.frombuffer(bytes(buf), dtype=np.uint8).reshape(h, w, 4).copy()
                    if args.flatten:
                        alpha = arr[:, :, 3:4].astype(np.float32) / 255.0
                        rgb = (arr[:, :, :3].astype(np.float32) * alpha).astype(np.uint8)
                        arr = np.dstack([rgb, np.full((h, w, 1), 255, np.uint8)])
                    from PIL import Image
                    Image.fromarray(arr, "RGBA").save(args.out)
                    print(f"tersimpan: {args.out} ({w}x{h}, delay {args.delay:.1f}s)")
                    recv.releaseReceiver()
                    recv.closeOpenGL()
                    glfw.terminate()
                    return 0
        time.sleep(0.005)

    print("FAIL: tidak ada frame")
    recv.releaseReceiver()
    recv.closeOpenGL()
    glfw.terminate()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
