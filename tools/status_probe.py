"""Record 0x800D status bytes over time through bus-error scenarios.

Needs the t2can-echo firmware on CAN A and its command FIFO (tools/t2can_console.py).
usage: python tools/status_probe.py <t2can-cmd-fifo>
"""
import sys
import threading
import time

import usb.core

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0])
from wscanfd.device import USBCANFD   # noqa: E402
from wscanfd.protocol import Frame    # noqa: E402

fifo = sys.argv[1]


def t2(cmd):
    with open(fifo, "w") as f:
        f.write(cmd + "\n")


def decode(b):
    ecr = int.from_bytes(b[4:8], "little")
    psr = int.from_bytes(b[8:12], "little")
    return (f"ECR: TEC={ecr & 0xFF:3d} REC={(ecr >> 8) & 0x7F:3d} RP={(ecr >> 15) & 1} CEL={(ecr >> 16) & 0xFF:3d} | "
            f"PSR: LEC={psr & 7} ACT={(psr >> 3) & 3} EP={(psr >> 5) & 1} EW={(psr >> 6) & 1} BO={(psr >> 7) & 1} "
            f"DLEC={(psr >> 8) & 7} TDCV={(psr >> 16) & 0x7F}")


def sample(d, label, secs, every=0.25):
    end = time.monotonic() + secs
    while time.monotonic() < end:
        b = d.status_raw(0)
        print(f"{label:22} {b[:16].hex(' ')}  {decode(b)}", flush=True)
        time.sleep(every)


def sender(d, stop, ident):
    n = 0
    while not stop.is_set():
        try:
            d.send(Frame(ident, n.to_bytes(8, "big")))
            n += 1
        except usb.core.USBError as e:
            print(f"   [send #{n} failed: {e}]", flush=True)
        time.sleep(0.05)


with USBCANFD() as d:
    t2("b 500000 2000000"); t2("h 1"); time.sleep(1)
    d.start_channel(0, 500_000, 2_000_000)
    sample(d, "healthy", 1.5, 0.5)

    t2("l 1"); time.sleep(0.5)                       # nobody ACKs
    stop = threading.Event()
    th = threading.Thread(target=sender, args=(d, stop, 0x55), daemon=True); th.start()
    sample(d, "no-ack, sending", 3)
    stop.set(); th.join()
    sample(d, "no-ack, stopped", 1.5, 0.5)
    t2("l 0"); time.sleep(1)
    sample(d, "acked again", 1.5, 0.5)

    # Wrong bitrate: T-2CAN at 250k (heartbeating), adapter at 500k sending
    t2("b 250000 2000000"); time.sleep(1)
    sample(d, "250k peer, rx only", 2, 0.5)
    stop = threading.Event()
    th = threading.Thread(target=sender, args=(d, stop, 0x66), daemon=True); th.start()
    sample(d, "250k peer, sending", 3)
    stop.set(); th.join()
    sample(d, "after", 2, 0.5)
    t2("b 500000 2000000"); time.sleep(1)
    sample(d, "peer back at 500k", 2, 0.5)
    n = 0
    while d.recv(0.05):
        n += 1
    print("frames received during probe:", n)
