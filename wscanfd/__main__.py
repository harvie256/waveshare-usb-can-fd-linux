"""wscanfd CLI.

  python -m wscanfd info
  python -m wscanfd dump [-c 0|1|both] [-b 250000] [-d 1000000] [--listen] [-t SECONDS]
  python -m wscanfd send [-c 0] [-b 250000] 123#1122334455667788   (cansend syntax, ## for FD)
  python -m wscanfd bridge [-c 0] [-b 250000] -i vcan0               (SocketCAN bridge)
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time

from .device import USBCANFD
from .protocol import Frame


def parse_cansend(s: str, channel: int) -> Frame:
    """cansend syntax: <id>#<data>, <id>#R[len], <id>##<flags><data> (FD; flags bit0 = BRS)."""
    ident, _, rest = s.partition("#")
    ext = len(ident) > 3
    can_id = int(ident, 16)
    if rest.startswith("#"):
        flags = int(rest[1], 16)
        data = bytes.fromhex(rest[2:].replace(".", ""))
        return Frame(can_id, data, extended=ext, fd=True, brs=bool(flags & 1), channel=channel)
    if rest.upper().startswith("R"):
        n = int(rest[1:] or 0)
        return Frame(can_id, bytes(n), extended=ext, rtr=True, channel=channel)
    return Frame(can_id, bytes.fromhex(rest.replace(".", "")), extended=ext, channel=channel)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="wscanfd")
    ap.add_argument("-v", "--verbose", action="count", default=0)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    for name in ("dump", "send", "bridge"):
        s = sub.add_parser(name)
        s.add_argument("-c", "--channel", default="0", help="0, 1 or both (dump only)")
        s.add_argument("-b", "--bitrate", type=int, default=250_000)
        s.add_argument("-d", "--data-bitrate", type=int, default=1_000_000)
        s.add_argument("--listen", action="store_true", help="listen-only mode")
        if name == "bridge":
            s.add_argument("-i", "--interface", default="vcan0")
        if name == "dump":
            s.add_argument("-t", "--time", type=float, default=0, help="stop after N seconds")
        elif name == "send":
            s.add_argument("frames", nargs="+")
            s.add_argument("-n", "--repeat", type=int, default=1)
            s.add_argument("-g", "--gap", type=float, default=0.01, help="seconds between frames")
            s.add_argument("-w", "--wait", type=float, default=0.5, help="listen for replies afterwards")
    a = ap.parse_args(argv)
    logging.basicConfig(level=[logging.WARNING, logging.INFO, logging.DEBUG][min(a.verbose, 2)],
                        format="%(levelname)s %(name)s: %(message)s")

    with USBCANFD() as dev:
        print(f"# {dev.info}", file=sys.stderr)
        if a.cmd == "info":
            print(dev.info.raw.hex())
            return 0
        chans = [0, 1] if a.channel == "both" and a.cmd == "dump" else [int(a.channel)]
        for ch in chans:
            dev.start_channel(ch, a.bitrate, a.data_bitrate, listen_only=a.listen)
        stop = [False]
        signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
        if a.cmd == "bridge":
            from . import bridge
            ev = threading.Event()
            signal.signal(signal.SIGINT, lambda *_: ev.set())
            signal.signal(signal.SIGTERM, lambda *_: ev.set())
            bridge.run(dev, chans[0], a.interface, ev)
            return 0
        if a.cmd == "send":
            for _ in range(a.repeat):
                for s in a.frames:
                    f = parse_cansend(s, chans[0])
                    dev.send(f)
                    print(f"TX {f}")
                    time.sleep(a.gap)
            deadline = time.monotonic() + a.wait
        else:
            deadline = time.monotonic() + a.time if a.time else float("inf")
        while not stop[0] and time.monotonic() < deadline:
            f = dev.recv(0.1)
            if f:
                print(f"RX {f}", flush=True)
        if dev.rx_dropped:
            print(f"# dropped {dev.rx_dropped} frames (queue full)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
