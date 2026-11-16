"""Round-trip test through a SocketCAN interface (kernel driver) against t2can-echo.

Each frame must come back with ID+1, data XOR 0xFF and the same length/FD/BRS/IDE.
usage: python3 tools/socketcan_roundtrip.py <ifname> [--classic-only] [--count N]
"""
import argparse
import socket
import struct
import sys
import time

DLC_TO_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
EFF, RTR = 0x80000000, 0x40000000
BRS, FDF = 0x01, 0x04


def cases(classic_only):
    n = 0
    for ext in (False, True):
        for length in range(9):
            yield ((0x200 + n) if not ext else (0x1800000 + n), ext, False, False,
                   bytes((n + i) & 0xFF for i in range(length)))
            n += 1
        if classic_only:
            continue
        for brs in (False, True):
            for length in DLC_TO_LEN:
                yield ((0x300 + n) if not ext else (0x1900000 + n), ext, True, brs,
                       bytes((n * 7 + i) & 0xFF for i in range(length)))
                n += 1


def pack(ident, ext, fd, brs, data):
    cid = ident | (EFF if ext else 0)
    if fd:
        return struct.pack("=IBBxx64s", cid, len(data), FDF | (BRS if brs else 0), data)
    return struct.pack("=IBxxx8s", cid, len(data), data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ifname")
    ap.add_argument("--classic-only", action="store_true")
    ap.add_argument("--gap", type=float, default=0.005)
    a = ap.parse_args()

    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
    s.bind((a.ifname,))
    s.settimeout(0.1)
    while True:
        try:
            s.recv(72)
        except socket.timeout:
            break

    sent = list(cases(a.classic_only))
    for c in sent:
        while True:
            try:
                s.send(pack(*c))
                break
            except OSError:          # ENOBUFS: queue full, back off
                time.sleep(0.002)
        time.sleep(a.gap)

    got = {}
    end = time.monotonic() + 2
    while time.monotonic() < end:
        try:
            b = s.recv(72)
        except socket.timeout:
            continue
        cid, length, flags = struct.unpack_from("=IBB", b)
        ext = bool(cid & EFF)
        ident = cid & (0x1FFFFFFF if ext else 0x7FF)
        if ident == 0x7E0:
            continue
        got[(ident, ext)] = (len(b) == 72, bool(flags & BRS) if len(b) == 72 else False, bytes(b[8:8 + length]))

    ok = bad = 0
    for ident, ext, fd, brs, data in sent:
        mask = 0x1FFFFFFF if ext else 0x7FF
        r = got.get(((ident + 1) & mask, ext))
        want = (fd, brs, bytes(x ^ 0xFF for x in data))
        if r != want:
            bad += 1
            print(f"FAIL id={ident:X} ext={ext} fd={fd} brs={brs} len={len(data)}: got {r}")
        else:
            ok += 1
    print(f"{a.ifname}: {ok}/{len(sent)} round-tripped OK, {bad} failed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
