"""Tes TUIO 2Dcur: dengarkan bundle /tuio/2Dcur selama beberapa detik.

    python tools/tuio_listen_test.py --port 3333 --seconds 4
"""
import argparse
import socket
import struct
import time


def _string_at(data: bytes, offset: int):
    end = data.index(b"\x00", offset)
    text = data[offset:end].decode("utf-8", "replace")
    nxt = end + 1
    nxt += (-nxt) % 4
    return text, nxt


def parse_message(data: bytes):
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


def parse_bundle(data: bytes):
    if not data.startswith(b"#bundle\x00"):
        return []
    pos = 8 + 8
    out = []
    while pos + 4 <= len(data):
        size = struct.unpack(">i", data[pos:pos + 4])[0]
        pos += 4
        msg = data[pos:pos + size]
        pos += size
        if not msg:
            break
        try:
            out.append(parse_message(msg))
        except Exception:
            continue
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=3333)
    ap.add_argument("--seconds", type=float, default=4.0)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", args.port))
    except OSError as exc:
        print(f"FAIL: tidak bisa bind port {args.port}: {exc}")
        return 2
    sock.settimeout(0.5)
    print(f"mendengarkan TUIO di 127.0.0.1:{args.port} selama {args.seconds:.0f} s ...")

    bundles = 0
    alive_frames = 0
    set_frames = 0
    fseq_values = []
    last_alive = []
    sample_set = None
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        try:
            data, _ = sock.recvfrom(65507)
        except socket.timeout:
            continue
        msgs = parse_bundle(data)
        if not msgs:
            continue
        bundles += 1
        for address, a in msgs:
            if address != "/tuio/2Dcur" or not a:
                continue
            kind = a[0]
            if kind == "alive":
                alive_frames += 1
                last_alive = [v for v in a[1:] if isinstance(v, int)]
            elif kind == "set":
                set_frames += 1
                if sample_set is None:
                    sample_set = a
            elif kind == "fseq":
                fseq_values.append(a[1] if len(a) > 1 else -1)
    sock.close()

    print(f"bundle diterima: {bundles}")
    print(f"alive: {alive_frames} frame, sesi terakhir: {last_alive}")
    print(f"set: {set_frames} pesan, contoh: {sample_set}")
    print(f"fseq: {fseq_values[:5]}{' ...' if len(fseq_values) > 5 else ''}")
    ok = bundles > 5 and alive_frames > 0 and set_frames > 0 and len(fseq_values) > 0
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
