"""Log the T-2CAN echo console to a file and forward commands from a FIFO.
usage: t2can_console.py <logfile> <fifo> [port]
then:  echo "b 500000 2000000" > <fifo>"""
import os, sys, threading, time, serial
log, fifo = sys.argv[1], sys.argv[2]
port = sys.argv[3] if len(sys.argv) > 3 else "/dev/ttyACM0"
if not os.path.exists(fifo):
    os.mkfifo(fifo)
time.sleep(1)
s = serial.Serial(port, 115200, timeout=0.5)
def fwd():
    while True:
        with open(fifo) as f:
            for line in f:
                s.write(line.strip().encode() + b"\n")
threading.Thread(target=fwd, daemon=True).start()
with open(log, "a", buffering=1) as out:
    while True:
        d = s.read(8192)
        if d:
            out.write(d.decode(errors="replace"))
