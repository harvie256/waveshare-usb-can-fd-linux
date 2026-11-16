#!/bin/sh
# Launch CANgaroo with the wscanfd test workspace.
# usage: tools/cangaroo/run.sh /path/to/CANgaroo-x86_64.AppImage [report-dir]
# Pre-configure the interface first, e.g.
#   sudo ip link set can1 type can bitrate 500000 sample-point 0.8 dbitrate 2000000 fd on && sudo ip link set can1 up
# then click Start in CANgaroo. If the setup dialog appears, tick "Configured by OS" for can1.
set -e
here=$(cd "$(dirname "$0")" && pwd)
app=${1:?path to CANgaroo AppImage or binary}
out=${2:-/tmp}
ws=$(mktemp --suffix=.cgw)
sed "s|@SCRIPT@|$here/wscanfd_test.py|" "$here/wscanfd-test.cgw.in" > "$ws"
WSCANFD_CANGAROO_OUT="$out" exec "$app" -w "$ws"
