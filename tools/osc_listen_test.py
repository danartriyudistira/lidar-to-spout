"""Tes OSC: dengarkan pesan dari aplikasi selama beberapa detik.

    python tools/osc_listen_test.py --port 9000 --seconds 4
"""
import argparse
import socket
import struct
import time


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--seconds", type=float, default=4.0)
    return ap.parse_args()


def _string_at(data: bytes, offset: int):
    end = data.index(b"\x00", offset)
    text = data[offset:end].decode("utf-8", "replace")
    nxt = end + 1
    nxt += (-nxt) % 4
    return text, nxt


def decode(data: bytes):
    address, off = _string_at(data, 0)
    tags, off = _string_at(data, off)
    payload = data[off:]
    vals = []
    for t in tags[1:]:
        if t == "f":
            vals.append(round(struct.unpack(">f", payload[:4])[0], 3))
            payload = payload[4:]
        elif t == "i":
            vals.append(struct.unpack(">i", payload[:4])[0])
            payload = payload[4:]
    return address, vals


def main() -> int:
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(0.5)
    print(f"mendengarkan OSC di 127.0.0.1:{args.port} selama {args.seconds:.0f} s ...")

    seen: dict[str, int] = {}
    t0 = time.time()
    sample: dict[str, list] = {}
    while time.time() - t0 < args.seconds:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        addr, vals = decode(data)
        seen[addr] = seen.get(addr, 0) + 1
        sample.setdefault(addr, vals)
    sock.close()
    if not seen:
        print("FAIL: tidak ada pesan OSC")
        return 2
    for addr in sorted(seen):
        print(f"{addr:28s} x{seen[addr]:<4} contoh={sample[addr]}")
    print("RESULT: PASS" if any(a.startswith("/zone/") for a in seen) else "RESULT: PARTIAL")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
