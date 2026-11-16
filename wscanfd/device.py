"""USB layer for the Waveshare USB-CAN-FD (non-B) using pyusb/libusb."""
from __future__ import annotations

import logging
import queue
import random
import threading
import time

import usb.core
import usb.util

from . import protocol as p

log = logging.getLogger("wscanfd")

VID, PID = 0x04D8, 0x0053
EP_CTRL_OUT = 0x01      # commands + all TX frames
EP_CTRL_IN = 0x81       # command replies
EP_RX = {0: 0x82, 1: 0x83}


class DeviceError(RuntimeError):
    pass


class USBCANFD:
    def __init__(self, serial: str | None = None):
        self._serial_wanted = serial
        self.dev: usb.core.Device | None = None
        self.info: p.DeviceInfo | None = None
        self.rx: "queue.Queue[p.Frame]" = queue.Queue(maxsize=100_000)
        self._wlock = threading.Lock()      # EP1 OUT is shared by commands and TX
        self._clock = threading.Lock()      # one command/reply exchange at a time
        self._threads: dict[int, threading.Thread] = {}
        self._running: dict[int, threading.Event] = {}
        self.rx_dropped = 0
        self._rates: dict[int, tuple[int, int | None]] = {}
        self._devq: dict[int, list[float]] = {}     # estimated wire-finish times per channel
        self._devq_lock = threading.Lock()

    # ------------------------------------------------------------ lifecycle

    def open(self) -> "USBCANFD":
        devs = list(usb.core.find(find_all=True, idVendor=VID, idProduct=PID))
        if not devs:
            raise DeviceError("no USB-CAN-FD (04d8:0053) found")
        self.dev = devs[0]      # TODO: select by serial once more than one is attached
        try:
            if self.dev.is_kernel_driver_active(0):
                self.dev.detach_kernel_driver(0)
        except (NotImplementedError, usb.core.USBError):
            pass
        # Never re-send SET_CONFIGURATION to a configured
        # device -- the firmware stops answering on EP1 IN afterwards.
        try:
            cfg = self.dev.get_active_configuration()
        except usb.core.USBError:
            cfg = None
        if cfg is None:
            self.dev.set_configuration()
        usb.util.claim_interface(self.dev, 0)
        self._drain(EP_CTRL_IN)
        self._authenticate()
        self.info = self.device_info()
        return self

    def close(self) -> None:
        if not self.dev:
            return
        for ch in list(self._threads):
            self.stop_channel(ch)
        try:
            self._command(p.Cmd.CLOSE, p.close_payload())
        except Exception as e:  # noqa: BLE001 - best effort on shutdown
            log.debug("close command failed: %s", e)
        usb.util.release_interface(self.dev, 0)
        usb.util.dispose_resources(self.dev)
        self.dev = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------ low level

    def _write(self, data: bytes, timeout: int = 1000) -> None:
        with self._wlock:
            n = self.dev.write(EP_CTRL_OUT, data, timeout)
        if n != len(data):
            raise DeviceError(f"short write {n}/{len(data)}")

    def _drain(self, ep: int, max_s: float = 0.1) -> None:
        # RX endpoints answer every poll (8 zero bytes when idle), so bound by time.
        end = time.monotonic() + max_s
        while time.monotonic() < end:
            try:
                self.dev.read(ep, 4096, 20)
            except usb.core.USBTimeoutError:
                return

    def _command(self, cmd: int, payload: bytes = b"", timeout: int = 3000) -> p.Packet:
        with self._clock:
            self._write(p.pack(cmd, payload))
            raw = bytes(self.dev.read(EP_CTRL_IN, 512, timeout))
        reply = p.unpack(raw)
        if reply.cmd != cmd:
            raise DeviceError(f"reply cmd {reply.cmd:#06x} != {cmd:#06x}")
        log.debug("cmd %#06x -> %s", cmd, reply.payload.hex())
        return reply

    def _authenticate(self) -> None:
        challenge = p.auth_challenge(0, int(time.time()), random.getrandbits(32))
        reply = self._command(p.Cmd.AUTH, challenge)
        if len(reply.payload) != len(challenge):
            raise DeviceError("unexpected reply to open command")

    # ------------------------------------------------------------ commands

    def device_info(self) -> p.DeviceInfo:
        return p.parse_device_info(self._command(p.Cmd.DEVICE_INFO).payload)

    def start_channel(self, ch: int, bitrate: int, data_bitrate: int = 1_000_000, *,
                      fd: bool = True, listen_only: bool = False, termination: bool = False,
                      non_iso: bool = False) -> None:
        if ch not in EP_RX:
            raise ValueError("channel must be 0 or 1")
        if ch in self._threads:
            self.stop_channel(ch)
        abit = p.timing_word(bitrate)
        dbit = p.timing_word(data_bitrate, data=True)
        log.info("ch%d: abit %d (%#010x) dbit %d (%#010x)", ch, bitrate, abit, data_bitrate, dbit)
        self._command(p.Cmd.INIT_CAN, p.init_payload(
            ch, abit, dbit, fd=fd, mode=p.MODE_LISTEN_ONLY if listen_only else p.MODE_NORMAL,
            termination=termination, non_iso=non_iso))
        self._command(p.Cmd.FILTER, p.clear_filter_payload(ch))
        self._drain(EP_RX[ch])
        self._command(p.Cmd.START_CAN, p.start_payload(ch))
        self._rates[ch] = (bitrate, data_bitrate if fd else None)
        self._devq[ch] = []
        ev = threading.Event()
        ev.set()
        t = threading.Thread(target=self._rx_loop, args=(ch, ev), daemon=True, name=f"wscanfd-rx{ch}")
        self._running[ch], self._threads[ch] = ev, t
        t.start()

    def stop_channel(self, ch: int) -> None:
        ev, t = self._running.pop(ch, None), self._threads.pop(ch, None)
        if ev:
            ev.clear()
        if t:
            t.join(1.0)
        try:
            self._command(p.Cmd.RESET_CAN, p.reset_payload(ch))
        except Exception as e:  # noqa: BLE001
            log.debug("reset ch%d failed: %s", ch, e)

    def status_raw(self, ch: int) -> bytes:
        """0x800D channel status (64 B). A stale snapshot, diagnostic only."""
        return self._command(p.Cmd.GET_STATUS, p.status_payload(ch)).payload

    def send(self, frame: p.Frame) -> None:
        """Queue one frame. Blocks as needed so the adapter's tiny TX buffer never
        overflows -- the firmware drops frames silently when it does."""
        nominal, data = self._rates.get(frame.channel, (250_000, None))
        dt = p.frame_time_s(frame, nominal, data)
        with self._devq_lock:
            q = self._devq.setdefault(frame.channel, [])
            now = time.monotonic()
            while q and q[0] <= now:
                q.pop(0)
            if len(q) >= p.TX_DEV_QUEUE:
                time.sleep(max(0.0, q[0] - now))
                now = time.monotonic()
                while q and q[0] <= now:
                    q.pop(0)
            q.append(max(now, q[-1] if q else now) + dt)
        self._write(p.pack(p.Cmd.TRANSMIT, p.transmit_payload(frame)))

    def recv(self, timeout: float | None = None) -> p.Frame | None:
        try:
            return self.rx.get(timeout=timeout)
        except queue.Empty:
            return None

    # ------------------------------------------------------------ RX

    def _rx_loop(self, ch: int, running: threading.Event) -> None:
        ep = EP_RX[ch]
        while running.is_set():
            try:
                buf = bytes(self.dev.read(ep, 4096, 200))
            except usb.core.USBTimeoutError:
                continue
            except usb.core.USBError as e:
                log.error("rx ch%d: %s", ch, e)
                time.sleep(0.1)
                continue
            for f in p.parse_rx(buf, ch):
                try:
                    self.rx.put_nowait(f)
                except queue.Full:
                    self.rx_dropped += 1
            if buf and len(buf) != 8 and not p.parse_rx(buf, ch):
                log.warning("rx ch%d: unparsed %d B: %s", ch, len(buf), buf[:64].hex())
