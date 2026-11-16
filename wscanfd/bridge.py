"""Bridge one USB-CAN-FD channel to a SocketCAN interface (normally vcan).

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan mtu 72   # mtu 72 = CAN FD capable
    sudo ip link set vcan0 up
    python -m wscanfd bridge -c 0 -i vcan0 -b 250000

Frames received on the bus appear on vcan0; frames sent to vcan0 (cansend,
python-can, ...) are transmitted on the bus. Bitrate/mode come from the CLI,
not from `ip link` -- that needs the kernel driver (option 2).
"""
from __future__ import annotations

import logging
import socket
import struct
import threading

from .device import USBCANFD
from .protocol import Frame

log = logging.getLogger("wscanfd.bridge")

CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG = 0x80000000, 0x40000000, 0x20000000
CANFD_BRS, CANFD_ESI, CANFD_FDF = 0x01, 0x02, 0x04
CAN_MTU, CANFD_MTU = 16, 72
SOL_CAN_RAW, CAN_RAW_FD_FRAMES = 101, 5


def to_socketcan(f: Frame) -> bytes:
    cid = f.can_id | (CAN_EFF_FLAG if f.extended else 0) | (CAN_RTR_FLAG if f.rtr else 0)
    if f.fd:
        flags = CANFD_FDF | (CANFD_BRS if f.brs else 0) | (CANFD_ESI if f.esi else 0)
        return struct.pack("=IBBxx64s", cid, len(f.data), flags, f.data)
    return struct.pack("=IBxxx8s", cid, len(f.data), f.data)


def from_socketcan(buf: bytes, channel: int) -> Frame | None:
    cid, length, flags = struct.unpack_from("=IBB", buf)
    if cid & CAN_ERR_FLAG:
        return None
    ext = bool(cid & CAN_EFF_FLAG)
    ident = cid & (0x1FFFFFFF if ext else 0x7FF)
    if len(buf) == CANFD_MTU:
        return Frame(ident, bytes(buf[8:8 + length]), extended=ext, fd=True,
                     brs=bool(flags & CANFD_BRS), channel=channel)
    rtr = bool(cid & CAN_RTR_FLAG)
    return Frame(ident, bytes(length) if rtr else bytes(buf[8:8 + length]),
                 extended=ext, rtr=rtr, channel=channel)


def run(dev: USBCANFD, channel: int, ifname: str, stop: threading.Event) -> None:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        s.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
    except OSError:
        log.warning("%s: CAN FD frames not supported (set mtu 72)", ifname)
    s.bind((ifname,))
    s.settimeout(0.2)
    counts = {"bus->sock": 0, "sock->bus": 0}

    def bus_to_sock():
        while not stop.is_set():
            f = dev.recv(0.2)
            if f is None or f.channel != channel:
                continue
            try:
                s.send(to_socketcan(f))
                counts["bus->sock"] += 1
            except OSError as e:
                log.warning("socket send: %s", e)

    t = threading.Thread(target=bus_to_sock, daemon=True)
    t.start()
    log.warning("bridging USB-CAN-FD ch%d <-> %s (Ctrl-C to stop)", channel, ifname)
    while not stop.is_set():
        try:
            buf = s.recv(CANFD_MTU)
        except socket.timeout:
            continue
        f = from_socketcan(buf, channel)
        if f:
            dev.send(f)
            counts["sock->bus"] += 1
    t.join(1)
    s.close()
    log.warning("bridge stopped: %s", counts)
