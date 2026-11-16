"""Round-trip test against the t2can-echo firmware on T-2CAN CAN A.

Every frame sent must come back with ID+1, data XOR 0xFF and the same
length / FD / BRS / IDE flags.

usage: python tools/fd_roundtrip.py [nominal] [data] [--classic-only] [--ch 0]
"""
from __future__ import annotations

import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0])
from wscanfd.device import USBCANFD          # noqa: E402
from wscanfd.protocol import DLC_TO_LEN, Frame  # noqa: E402


def cases(classic_only: bool):
    n = 0
    for ext in (False, True):
        for length in range(9):
            yield Frame((0x200 + n) if not ext else (0x1800000 + n), bytes((n + i) & 0xFF for i in range(length)),
                        extended=ext)
            n += 1
        if classic_only:
            continue
        for brs in (False, True):
            for length in DLC_TO_LEN:
                yield Frame((0x300 + n) if not ext else (0x1900000 + n),
                            bytes((n * 7 + i) & 0xFF for i in range(length)), extended=ext, fd=True, brs=brs)
                n += 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("nominal", type=int, nargs="?", default=250_000)
    ap.add_argument("data", type=int, nargs="?", default=2_000_000)
    ap.add_argument("--classic-only", action="store_true")
    ap.add_argument("--ch", type=int, default=0)
    ap.add_argument("--gap", type=float, default=0.01)
    a = ap.parse_args()

    sent = list(cases(a.classic_only))
    with USBCANFD() as d:
        d.start_channel(a.ch, a.nominal, a.data)
        time.sleep(0.5)
        while d.recv(0.05):
            pass
        for f in sent:
            f.channel = a.ch
            d.send(f)
            time.sleep(a.gap)
        got = {}
        end = time.monotonic() + 2
        while time.monotonic() < end:
            r = d.recv(0.1)
            if r and r.can_id != 0x7E0:
                got[(r.can_id, r.extended)] = r

    ok = bad = 0
    for f in sent:
        mask = 0x1FFFFFFF if f.extended else 0x7FF
        r = got.get(((f.can_id + 1) & mask, f.extended))
        want = bytes(b ^ 0xFF for b in f.data)
        kind = ("FD" + ("+BRS" if f.brs else "")) if f.fd else "CAN"
        if r is None:
            print(f"MISSING  {kind:7} {f}")
            bad += 1
        elif r.data != want or r.fd != f.fd or r.brs != f.brs:
            print(f"MISMATCH {kind:7} sent {f}\n                 got  {r} (fd={r.fd} brs={r.brs})")
            bad += 1
        else:
            ok += 1
    print(f"{a.nominal}/{a.data}: {ok}/{len(sent)} round-tripped OK, {bad} failed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
