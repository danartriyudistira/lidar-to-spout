"""Uji gabungan: satu sesi aplikasi -> Spout + OSC + TUIO sekaligus.

    python tools/smoke_all.py                      # hardware + config.json
    python tools/smoke_all.py --replay data/live_rec.bin --seconds 4
    python tools/smoke_all.py --config lain.json --no-spout
"""
from __future__ import annotations

import argparse
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from config import load_config  # noqa: E402


# ------------------------------------------------------------------ OSC / TUIO
def _string_at(data: bytes, offset: int):
    end = data.index(b"\x00", offset)
    text = data[offset:end].decode("utf-8", "replace")
    nxt = end + 1
    nxt += (-nxt) % 4
    return text, nxt


def _parse_osc(data: bytes):
    address, off = _string_at(data, 0)
    tags, off = _string_at(data, off)
    args = []
    payload = data[off:]
    for tag in tags[1:]:
        if tag == "i":
            args.append(struct.unpack(">i", payload[:4])[0])
            payload = payload[4:]
        elif tag == "f":
            args.append(round(struct.unpack(">f", payload[:4])[0], 4))
            payload = payload[4:]
        elif tag == "s":
            text, consumed = _string_at(payload, 0)
            args.append(text)
            payload = payload[consumed:]
    return address, args


def _parse_tuio_bundle(data: bytes):
    if not data.startswith(b"#bundle\x00"):
        return []
    pos = 16
    out = []
    while pos + 4 <= len(data):
        size = struct.unpack(">i", data[pos:pos + 4])[0]
        pos += 4
        msg = data[pos:pos + size]
        pos += size
        if not msg:
            break
        try:
            out.append(_parse_osc(msg))
        except Exception:
            pass
    return out


def osc_worker(port: int, stop: threading.Event, result: dict) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", port))
    except OSError as exc:
        result["error"] = f"tidak bisa bind OSC {port}: {exc}"
        return
    sock.settimeout(0.3)
    seen = {}
    while not stop.is_set():
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        try:
            addr, args = _parse_osc(data)
        except Exception:
            continue
        seen[addr] = seen.get(addr, 0) + 1
    sock.close()
    result["messages"] = seen
    result["count"] = sum(seen.values())


def tuio_worker(port: int, stop: threading.Event, result: dict) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", port))
    except OSError as exc:
        result["error"] = f"tidak bisa bind TUIO {port}: {exc}"
        return
    sock.settimeout(0.3)
    bundles = sets = alive = 0
    last_sessions: list[int] = []
    max_sessions = 0
    fseq = []
    while not stop.is_set():
        try:
            data, _ = sock.recvfrom(65507)
        except socket.timeout:
            continue
        msgs = _parse_tuio_bundle(data)
        if msgs:
            bundles += 1
        for addr, args in msgs:
            if addr != "/tuio/2Dcur" or not args:
                continue
            kind = args[0]
            if kind == "alive":
                alive += 1
                ids = [v for v in args[1:] if isinstance(v, int)]
                if ids:
                    last_sessions = ids
                    max_sessions = max(max_sessions, len(ids))
            elif kind == "set":
                sets += 1
            elif kind == "fseq" and len(args) > 1:
                fseq.append(args[1])
    sock.close()
    result.update(bundles=bundles, sets=sets, alive=alive,
                  sessions=last_sessions, max_sessions=max_sessions, fseq=fseq[-3:], count=bundles)


# ------------------------------------------------------------------ Spout
def spout_check(name: str, seconds: float, result: dict) -> None:
    try:
        import glfw
        import moderngl
        import numpy as np
        import SpoutGL
    except Exception as exc:
        result["error"] = f"dependency Spout tidak lengkap: {exc}"
        return

    if not glfw.init():
        result["error"] = "glfw init gagal"
        return
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    win = glfw.create_window(64, 64, "smoke", None, None)
    if not win:
        result["error"] = "window GL gagal"
        return
    glfw.make_context_current(win)
    moderngl.create_context()

    recv = SpoutGL.SpoutReceiver()
    recv.setReceiverName(name)
    recv.createOpenGL()

    frames = lit = 0
    maxv = 0
    w = h = 0
    buf = None
    t0 = time.time()
    while time.time() - t0 < seconds:
        if recv.receiveTexture():
            frames += 1
            w = recv.getSenderWidth()
            h = recv.getSenderHeight()
            need = w * h * 4
            if buf is None or len(buf) != need:
                buf = bytearray(need)
            if recv.receiveImage(buf, 6408, False, 0):
                arr = np.frombuffer(bytes(buf), dtype=np.uint8)
                if arr.max() > 8:
                    lit += 1
                maxv = max(maxv, int(arr.max()))
        else:
            time.sleep(0.005)
    recv.releaseReceiver()
    recv.closeOpenGL()
    glfw.terminate()
    result.update(frames=frames, lit=lit, width=w, height=h, maxv=maxv, count=frames)


# ------------------------------------------------------------------ main
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "config.json"))
    ap.add_argument("--replay", default="")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--no-spout", action="store_true")
    args = ap.parse_args()

    cfg = load_config(args.config)
    spout_name = cfg["spout"].get("beauty", "Lidar Beauty")
    osc_port = int(cfg["osc"].get("port", 9000))
    tuio_port = int(cfg["tuio"].get("port", 3333))

    stop = threading.Event()
    osc_res: dict = {}
    tuio_res: dict = {}
    threads = [
        threading.Thread(target=osc_worker, args=(osc_port, stop, osc_res), daemon=True),
        threading.Thread(target=tuio_worker, args=(tuio_port, stop, tuio_res), daemon=True),
    ]
    for t in threads:
        t.start()

    cmd = [sys.executable, str(ROOT / "src" / "app.py"), "--config", args.config, "--headless"]
    if args.replay:
        cmd += ["--replay", str(ROOT / args.replay) if not Path(args.replay).is_absolute() else args.replay,
                "--speed", str(args.speed)]
    if args.no_spout:
        cmd += ["--no-spout"]
    print("menjalankan:", " ".join(cmd[1:]))
    app = subprocess.Popen(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    time.sleep(2.5)
    spout_res: dict = {}
    if args.no_spout:
        spout_res["count"] = 0
        spout_res["skipped"] = True
    else:
        spout_check(spout_name, args.seconds, spout_res)

    stop.set()
    for t in threads:
        t.join(timeout=2.0)

    app.terminate()
    try:
        out, _ = app.communicate(timeout=5)
    except Exception:
        app.kill()
        out = ""
    tail = (out or "").strip().splitlines()[-3:]
    if tail:
        print("log app:", " | ".join(tail))

    print("\n=== hasil ===")
    ok = True
    if spout_res.get("skipped"):
        print("Spout : SKIP")
    else:
        sp = spout_res.get("frames", 0) and spout_res.get("lit", 0)
        print(f"Spout : {spout_res.get('frames', 0)} frame, berisi {spout_res.get('lit', 0)}, "
              f"{spout_res.get('width')}x{spout_res.get('height')} "
              f"{'PASS' if sp else 'FAIL ' + str(spout_res.get('error', ''))}")
        ok = ok and bool(sp)
    print(f"OSC   : {osc_res.get('count', 0)} pesan {osc_res.get('error', '')} "
          f"{'PASS' if osc_res.get('count', 0) > 5 else 'FAIL'}")
    ok = ok and osc_res.get("count", 0) > 5
    print(f"TUIO  : {tuio_res.get('bundles', 0)} bundle, {tuio_res.get('sets', 0)} set, "
          f"sesi {tuio_res.get('sessions', [])} {tuio_res.get('error', '')} "
          f"{'PASS' if tuio_res.get('bundles', 0) > 5 and tuio_res.get('alive', 0) > 0 else 'FAIL'}")
    ok = ok and tuio_res.get("bundles", 0) > 5 and tuio_res.get("alive", 0) > 0

    if osc_res.get("messages"):
        top = sorted(osc_res["messages"].items(), key=lambda kv: -kv[1])[:4]
        print("OSC teratas:", ", ".join(f"{k} x{v}" for k, v in top))
    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
