"""Wire protocol for the Waveshare USB-CAN-FD (04d8:0053, NXP LPC54616 firmware).

Pure functions only (no USB I/O), so it can be unit-tested against captures.
See docs/index.html for the protocol reference.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

# ---------------------------------------------------------------- framing

HEAD = 0xBEEF
TAIL = 0xDEAD
CHECK_BIAS = 0xBEAD
MAX_PAYLOAD = 500


class Cmd:
    AUTH = 0x8001
    INIT_CAN = 0x8002
    START_CAN = 0x8003
    TRANSMIT = 0x8004
    DEVICE_INFO = 0x8005
    GET_USER_SN = 0x8007
    SET_USER_SN = 0x8008
    RESET_CAN = 0x8009
    CLOSE = 0x800A
    FILTER = 0x800B
    CLEAR_BUFFER = 0x800C
    GET_STATUS = 0x800D     # channel status snapshot (not live)
    # 0x8006 = Firmware_Update (enters bootloader?) -- never send


def checksum(cmd: int, payload: bytes, pkgall: int = 1, pkgcur: int = 1) -> int:
    c = pkgcur * 256 + pkgall * 256 + len(payload) + 2 * cmd
    for i in range(len(payload) // 2):          # odd trailing byte is not summed
        c += payload[2 * i] * 256 + payload[2 * i + 1]
    return (c + CHECK_BIAS) & 0xFFFF


def pack(cmd: int, payload: bytes = b"", pkgall: int = 1, pkgcur: int = 1) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    return (struct.pack(">HHBBH", HEAD, len(payload), pkgall, pkgcur, cmd) + payload
            + struct.pack(">HH", checksum(cmd, payload, pkgall, pkgcur), TAIL))


@dataclass
class Packet:
    cmd: int
    payload: bytes
    pkgall: int = 1
    pkgcur: int = 1


class FrameError(ValueError):
    pass


def unpack(buf: bytes) -> Packet:
    if len(buf) < 12:
        raise FrameError(f"short packet ({len(buf)} B)")
    head, n, pkgall, pkgcur, cmd = struct.unpack_from(">HHBBH", buf)
    if head != HEAD:
        raise FrameError(f"bad head {head:#06x}")
    if n > MAX_PAYLOAD or len(buf) < 12 + n:
        raise FrameError(f"bad length {n} for {len(buf)} B packet")
    payload = bytes(buf[8:8 + n])
    check, tail = struct.unpack_from(">HH", buf, 8 + n)
    if tail != TAIL:
        raise FrameError(f"bad tail {tail:#06x}")
    if check != checksum(cmd, payload, pkgall, pkgcur):
        raise FrameError("bad checksum")
    return Packet(cmd, payload, pkgall, pkgcur)


# ---------------------------------------------------------------- auth

# Open handshake: the host sends a 32-byte challenge (fixed template with an
# address, time and random field); the device replies with 32 bytes, which is
# not validated.
AUTH_TEMPLATE = bytes.fromhex(
    "deff003faf674301f28982eece2f2216d01f07115f68b3b12ce9c23ec8265236")


def auth_challenge(addr: int, now: int, rnd: int) -> bytes:
    c = bytearray(AUTH_TEMPLATE)
    c[2:6] = struct.pack(">I", addr & 0xFFFFFFFF)
    c[12:16] = struct.pack(">I", now & 0xFFFFFFFF)
    c[22:26] = struct.pack(">I", rnd & 0xFFFFFFFF)
    return bytes(c)




# ---------------------------------------------------------------- device info

@dataclass
class DeviceInfo:
    hw_version: int
    fw_version: int
    serial: str
    raw: bytes

    def __str__(self) -> str:
        return (f"serial={self.serial} hw={self.hw_version >> 8}.{self.hw_version & 0xFF:02x} "
                f"fw={self.fw_version >> 8}.{self.fw_version & 0xFF:02x}")


def parse_device_info(payload: bytes) -> DeviceInfo:
    # Observed: 02 00 | 02 12 | 00 00 00 02 | "USBCANFD2024..." (20 chars) ...
    hw, fw = struct.unpack_from(">HH", payload)
    serial = payload[8:28].split(b"\0")[0].decode("ascii", "replace")
    return DeviceInfo(hw, fw, serial, payload)


# ---------------------------------------------------------------- bit timing

CAN_CLOCK = 60_000_000      # M_CAN clock on this board


@dataclass(frozen=True)
class BitTiming:
    """NXP MCAN NBTP/DBTP fields (actual values, not minus-one)."""
    brp: int
    tseg1: int      # prop + phase1
    tseg2: int
    sjw: int

    def word(self) -> int:
        return ((self.brp - 1) << 24) | ((self.sjw - 1) << 16) | ((self.tseg1 - 1) << 8) | (self.tseg2 - 1)

    @classmethod
    def from_word(cls, w: int) -> "BitTiming":
        return cls(((w >> 24) & 0xFF) + 1, ((w >> 8) & 0xFF) + 1, (w & 0xFF) + 1, ((w >> 16) & 0xFF) + 1)

    @property
    def tq(self) -> int:
        return 1 + self.tseg1 + self.tseg2

    def bitrate(self, clock: int = CAN_CLOCK) -> float:
        return clock / (self.brp * self.tq)

    def sample_point(self) -> float:
        return (1 + self.tseg1) / self.tq


# Known-good timing words (80 % sample point). Used verbatim when the rate matches.
REF_ABIT = {
    1_000_000: 0x00022E0B, 800_000: 0x00023A0E, 500_000: 0x00025E17, 250_000: 0x0014BE2F,
    125_000: 0x0114BE2F, 100_000: 0x0314761D, 50_000: 0x0314EE3B,
}
REF_DBIT = {
    5_000_000: 0x00010702, 4_000_000: 0x00010A02, 3_000_000: 0x01000601, 2_500_000: 0x01010702,
    2_000_000: 0x01010A02, 1_000_000: 0x03010A02, 800_000: 0x04010A02, 500_000: 0x05020E03,
    250_000: 0x0B020E03, 100_000: 0x1D020E03,
}
DEFAULT_DBIT = 0x03030A02   # default data timing, 1 Mbit/s


def calc_timing(bitrate: int, sample_point: float = 0.8, *, data: bool = False,
                clock: int = CAN_CLOCK, tolerance: float = 0.005) -> BitTiming:
    """Pick MCAN timing for a bitrate. Nominal: brp<=256, tseg1<=256, tseg2<=128;
    data: brp<=32, tseg1<=32, tseg2<=16."""
    max_brp, max_t1, max_t2 = (32, 32, 16) if data else (256, 256, 128)
    best = None
    for brp in range(1, max_brp + 1):
        tq = clock / (brp * bitrate)
        ntq = round(tq)
        if ntq < 8 or abs(tq - ntq) / tq > tolerance:
            continue
        t1 = round(ntq * sample_point) - 1
        t2 = ntq - 1 - t1
        if not (1 <= t1 <= max_t1 and 1 <= t2 <= max_t2):
            continue
        cand = BitTiming(brp, t1, t2, min(t2, 16 if data else 128))
        score = (abs(cand.bitrate(clock) - bitrate) / bitrate > 1e-9,
                 abs(cand.sample_point() - sample_point), brp)
        if best is None or score < best[0]:
            best = (score, cand)
    if best is None:
        raise ValueError(f"no timing within {tolerance:.1%} for {bitrate} bit/s at {clock} Hz")
    return best[1]


def timing_word(bitrate: int, *, data: bool = False) -> int:
    table = REF_DBIT if data else REF_ABIT
    if bitrate in table:
        return table[bitrate]
    return calc_timing(bitrate, data=data).word()


# ---------------------------------------------------------------- commands

MODE_NORMAL = 0
MODE_LISTEN_ONLY = 1       # confirmed on hardware
DEVICE_TYPE = 41           # device type byte used in reset/status payloads


def init_payload(ch: int, abit: int, dbit: int = DEFAULT_DBIT, *, fd: bool = True,
                 mode: int = MODE_NORMAL, termination: bool = False,
                 non_iso: bool = False) -> bytes:
    """0x8002 payload (32 B). abit/dbit are MCAN timing words."""
    b = bytearray(32)
    b[0:4] = bytes([0x55, 0x02, 1 if ch else 0, 1 if fd else 0])
    b[4] = mode
    b[5] = 1                                    # "single filter"
    b[6:10] = struct.pack(">I", 0)              # acc_code
    b[10:14] = struct.pack(">I", 0xFFFFFFFF)    # acc_mask
    b[18:22] = struct.pack(">I", abit)
    b[22:26] = struct.pack(">I", dbit)
    b[26:30] = struct.pack(">I", 0)             # brp field, unused (0)
    # 0x40 = termination request; a no-op on this board (termination is a physical switch)
    b[30] = (0x80 if non_iso else 0) | (0x40 if termination else 0)
    return bytes(b)


def start_payload(ch: int) -> bytes:
    return bytes([0x55, 0x80, 0x03, 1 if ch else 0])


def reset_payload(ch: int, dev_index: int = 0) -> bytes:
    return bytes([0x55, 0x09, DEVICE_TYPE, dev_index, 1 if ch else 0, 0, 0, 0])


def status_payload(ch: int, dev_index: int = 0) -> bytes:
    return bytes([0x55, 0x0D, DEVICE_TYPE, dev_index, 1 if ch else 0, 0, 0, 0])


def close_payload() -> bytes:
    return bytes([0x55, 0x0A])


def clear_filter_payload(ch: int) -> bytes:
    return bytes([0x7E, 1 if ch else 0, 0x0F, 0xFF])


# ---------------------------------------------------------------- frames

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000

DLC_TO_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]


def len_to_dlc(n: int) -> int:
    for dlc, size in enumerate(DLC_TO_LEN):
        if size >= n:
            return dlc
    raise ValueError("CAN FD payload > 64 bytes")


@dataclass
class Frame:
    can_id: int                 # 11- or 29-bit identifier, no flag bits
    data: bytes = b""
    extended: bool = False
    rtr: bool = False
    fd: bool = False
    brs: bool = False
    esi: bool = False
    channel: int = 0
    timestamp_us: int | None = None
    filter_index: int | None = None
    raw: bytes = field(default=b"", repr=False)

    def __str__(self) -> str:
        ident = f"{self.can_id:08X}" if self.extended else f"{self.can_id:03X}"
        if self.rtr:
            body = f"R{len(self.data) or ''}"
        elif self.fd:
            body = ("B" if self.brs else "") + f"[{len(self.data):02d}] " + " ".join(f"{b:02X}" for b in self.data)
        else:
            body = f"[{len(self.data)}] " + " ".join(f"{b:02X}" for b in self.data)
        ts = f"({self.timestamp_us / 1e6:12.4f}) " if self.timestamp_us is not None else ""
        return f"{ts}can{self.channel} {ident}  {body}"


def transmit_payload(f: Frame) -> bytes:
    ident = f.can_id & 0x1FFFFFFF
    if f.fd:
        if len(f.data) > 64:
            raise ValueError("CAN FD payload > 64 bytes")
        n = DLC_TO_LEN[len_to_dlc(len(f.data))]
        b = bytearray(0x56)
        b[0:4] = bytes([0x55, 0xF2, 0, 0])
        b[4], b[5] = int(f.extended), int(f.rtr)
        b[6:10] = struct.pack(">I", ident)
        b[10] = n
        b[11] = 1 if f.brs else 0          # canfd_frame.flags bit0 = BRS (confirmed)
        b[12] = 1
        b[13] = f.channel
        b[14:14 + len(f.data)] = f.data
        b[0x51] = 0                         # transmit_type 0 = normal (auto-retransmit)
        return bytes(b)
    if len(f.data) > 8:
        raise ValueError("classic CAN payload > 8 bytes")
    b = bytearray(0x1A)
    b[0:4] = bytes([0x55, 0xF1, 0, 0])
    b[4], b[5] = int(f.extended), int(f.rtr)
    b[6:10] = struct.pack(">I", ident)
    b[10] = len(f.data)
    b[11:11 + len(f.data)] = f.data
    b[0x14] = f.channel
    b[0x15] = 0
    return bytes(b)


CLASSIC_REC = 21    # R0 R1 data[8] 'U' ts32le
FD_REC = 76         # R0 R1 data[64] ts32le
TS_TICK_US = 100    # device timestamp tick


def parse_rx(buf: bytes, channel: int = 0) -> list[Frame]:
    """Parse one bulk IN transfer from EP 0x82/0x83 into frames.

    Each record starts with an NXP MCAN RX element header:
      R0: id[28:0] (std id in [28:18]), RTR bit 29, XTD bit 30, ESI bit 31
      R1: rxts[15:0], DLC[19:16], BRS 20, FDF 21, FIDX[30:24], ANMF 31
    Idle polls are 8 zero bytes and yield nothing."""
    frames: list[Frame] = []
    pos = 0
    n = len(buf)
    while pos + 8 <= n:
        r0, r1 = struct.unpack_from("<II", buf, pos)
        fdf = bool(r1 & (1 << 21))
        size = FD_REC if fdf else CLASSIC_REC
        if pos + size > n:
            break
        if not fdf and buf[pos + 16] != 0x55:
            break                               # not a record (e.g. idle 8 x 00)
        xtd = bool(r0 & (1 << 30))
        rtr = bool(r0 & (1 << 29))
        ident = r0 & 0x1FFFFFFF if xtd else (r0 >> 18) & 0x7FF
        dlc = (r1 >> 16) & 0xF
        length = DLC_TO_LEN[dlc] if fdf else min(dlc, 8)
        data = bytes(buf[pos + 8:pos + 8 + length]) if not rtr else b""
        ts_off = pos + (72 if fdf else 17)
        ts = struct.unpack_from("<I", buf, ts_off)[0] * TS_TICK_US
        frames.append(Frame(ident, data, extended=xtd, rtr=rtr, fd=fdf,
                            brs=bool(r1 & (1 << 20)), esi=bool(r0 & (1 << 31)),
                            channel=channel, timestamp_us=ts,
                            filter_index=(r1 >> 24) & 0x7F, raw=bytes(buf[pos:pos + size])))
        pos += size
    return frames


# ---------------------------------------------------------------- TX pacing

TX_DEV_QUEUE = 4            # frames we allow inside the adapter (it holds ~10, then drops)
TX_MIN_FRAME_US = 300       # firmware per-frame cost: ~300 us + 2 us/byte (fw 2.12)


def frame_time_s(f: "Frame", nominal: int, data: int | None) -> float:
    """Worst-case wire time (+10%), floored at the firmware's per-frame cost."""
    n = len(f.data)
    if f.fd:
        hdr = 30 + (18 if f.extended else 0)             # SOF..DLC, incl. FD control bits
        tail = 28 + 10                                    # CRC21+stuff, delims, ACK, EOF, IFS
        bits_nom = hdr + 12
        bits_data = n * 8 * 1.25 + tail
        t = (bits_nom / nominal + bits_data / data) if (f.brs and data) else (bits_nom + bits_data) / nominal
    else:
        bits = (34 if not f.extended else 54) + n * 8
        t = (bits * 1.2 + 13) / nominal                   # stuffing + EOF/IFS
    return max(t * 1.1, (TX_MIN_FRAME_US + 2 * n) * 1e-6)
