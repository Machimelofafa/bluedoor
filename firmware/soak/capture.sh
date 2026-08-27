#!/usr/bin/env bash
# Overnight serial capture for the stability soak (reset hunt, 2026-08-27).
#
# Usage:  ./capture.sh [port]        default port: /dev/ttyUSB0
#
# Timestamps every line and appends to soak/serial-<stamp>.log. Also snapshots
# the current logger build's firmware.elf next to the log, so backtrace PCs in
# a crash dump stay decodable after later rebuilds:
#   xtensa-esp32-elf-addr2line -pfiaC -e firmware-<stamp>.elf <pc> <pc> ...
# (the toolchain lives under ~/.platformio/packages/toolchain-xtensa-esp32/bin)
#
# Raw capture on purpose — no pio monitor filters — so nothing the panic
# handler prints is ever eaten. Leave it running past the crash: the decisive
# lines (Guru Meditation / "Interrupt wdt timeout on CPU0" / task_wdt) arrive
# in the second before the reboot.
set -euo pipefail

PORT="${1:-/dev/ttyUSB0}"
DIR="$(cd "$(dirname "$0")" && pwd)"
STAMP="$(date +%Y%m%d-%H%M)"
LOG="$DIR/serial-$STAMP.log"
ELF="$DIR/../.pio/build/logger/firmware.elf"

[ -e "$PORT" ] || { echo "no $PORT — is the board plugged in?"; exit 1; }
[ -f "$ELF" ] && cp "$ELF" "$DIR/firmware-$STAMP.elf"

# The CH340 can fall off the bus and re-enumerate (seen live 22:35 on soak
# night one: error -71, host power-cycled the port, reader got EOF after 14
# minutes). Survive it: reopen forever, and re-run stty every time — a fresh
# enumeration resets the port to 9600 baud. clocal so a modem-line hiccup
# can't hang up the reader.
echo "capturing $PORT -> $LOG  (ctrl-c to stop; survives USB re-enumeration)"
while true; do
  if [ ! -e "$PORT" ] || ! stty -F "$PORT" 115200 raw -echo -echoe -echok clocal 2>/dev/null; then
    sleep 2
    continue
  fi
  python3 -u -c '
import sys, datetime
try:
    for line in sys.stdin.buffer:
        stamp = datetime.datetime.now().strftime("%m-%d %H:%M:%S.%f")[:-3]
        sys.stdout.buffer.write(stamp.encode() + b" " + line)
        sys.stdout.buffer.flush()
except OSError:
    pass
' < "$PORT" >> "$LOG" || true
  MARK="$(date '+%m-%d %H:%M:%S') CAPTURE port dropped (USB re-enumeration?) — reopening"
  echo "$MARK" >> "$LOG"
  echo "$MARK"
  sleep 2
done
