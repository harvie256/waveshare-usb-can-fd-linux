"""CANgaroo AutoRun script: round-trip test of the wscanfd kernel driver against t2can-echo.

Loaded by tools/cangaroo/wscanfd-test.cgw (AutoRun). Start the measurement in
CANgaroo; results go to the CANgaroo log window and to REPORT below.
The echo peer returns every frame with ID+1 and data XOR 0xFF, keeping FD/BRS/IDE.
"""
import os
import time

import cangaroo

OUT_DIR = os.environ.get("WSCANFD_CANGAROO_OUT", "/tmp")
REPORT = os.path.join(OUT_DIR, "cangaroo-report.txt")
TRACE = os.path.join(OUT_DIR, "cangaroo-trace.asc")
IFNAME = "can1"
DLC_TO_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]

lines = []


def out(s):
    lines.append(s)
    print(s)
    with open(REPORT, "w") as f:
        f.write("\n".join(lines) + "\n")


def cases():
    n = 0
    for ext in (False, True):
        for length in range(9):
            yield ((0x200 + n) if not ext else (0x1800000 + n), ext, False, False,
                   bytes((n + i) & 0xFF for i in range(length)))
            n += 1
        for brs in (False, True):
            for length in DLC_TO_LEN:
                yield ((0x300 + n) if not ext else (0x1900000 + n), ext, True, brs,
                       bytes((n * 7 + i) & 0xFF for i in range(length)))
                n += 1


def make(ident, ext, fd, brs, data):
    m = cangaroo.Message()
    m.id = ident
    m.extended = ext
    m.fd = fd
    m.brs = brs
    m.set_data(data)
    return m


def collect(secs):
    got = []
    end = time.time() + secs
    while time.time() < end:
        got.extend(m for m in cangaroo.receive(0.05) if m.is_rx)
    return got


out(f"CANgaroo wscanfd test, {time.strftime('%Y-%m-%d %H:%M:%S')}")
ifaces = cangaroo.interfaces()
out(f"interfaces: {ifaces}")
iid = next((i["id"] for i in ifaces if i["name"] == IFNAME), None)
if iid is None:
    out(f"RESULT: FAIL - {IFNAME} not in measurement setup")
    raise SystemExit
out(f"{IFNAME}: id={iid} state={cangaroo.interface_state(iid)} caps={cangaroo.interface_capabilities(iid)}")

collect(0.5)                      # drop anything already queued

# 1) round trip, every frame type/length
sent = list(cases())
for c in sent:
    cangaroo.send(make(*c), iid)
    time.sleep(0.01)
rx = collect(2.0)
by_key = {(m.id, m.extended): m for m in rx}
ok = 0
for ident, ext, fd, brs, data in sent:
    r = by_key.get(((ident + 1) & (0x1FFFFFFF if ext else 0x7FF), ext))
    want = bytes(b ^ 0xFF for b in data)
    if r is not None and r.get_data() == want and r.fd == fd and r.brs == brs:
        ok += 1
    else:
        got = None if r is None else (r.get_data().hex(), r.fd, r.brs)
        out(f"  FAIL id={ident:X} ext={ext} fd={fd} brs={brs} len={len(data)} got={got}")
out(f"round-trip: {ok}/{len(sent)} OK")
hb = [m for m in rx if m.id == 0x7E0]
out(f"heartbeats from peer seen: {len(hb)}")
ts = [m.timestamp for m in rx]
out(f"rx timestamps monotonic: {all(a <= b for a, b in zip(ts, ts[1:]))}")

# 2) burst: 500 back-to-back 64-byte FD+BRS frames
N = 500
t0 = time.time()
for i in range(N):
    cangaroo.send(make(0x100, False, True, True, i.to_bytes(2, "big") + bytes(62)), iid)
t_send = time.time() - t0
rx = collect(3.0)
seqs = {int.from_bytes(bytes(b ^ 0xFF for b in m.get_data()[:2]), "big") for m in rx if m.id == 0x101}
out(f"burst: {N} FD64+BRS queued in {t_send:.2f}s, echoes back {len(seqs)}/{N}")

out(f"interface state after tests: {cangaroo.interface_state(iid)}")
out(f"trace size: {cangaroo.trace_size()}")
try:
    cangaroo.save_trace(TRACE)
    out(f"trace saved: {TRACE}")
except Exception as e:  # noqa: BLE001
    out(f"save_trace failed: {e}")
out("RESULT: " + ("PASS" if ok == len(sent) and len(seqs) == N else "FAIL"))
