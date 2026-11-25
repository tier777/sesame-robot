#!/usr/bin/env bash
# Build and (optionally) flash Sesame on a plain ESP32 DevKit with arduino-cli.
#   bash tools/flash.sh main                      # compile the main firmware
#   bash tools/flash.sh tester /dev/cu.usbserial-110  # compile and upload the motor tester
# Arduino wants the sketch folder named like the .ino, so the files are copied into one first.
set -euo pipefail
cd "$(dirname "$0")/.."

which="${1:-main}"
port="${2:-}"
fqbn="${FQBN:-esp32:esp32:esp32}"
work="${TMPDIR:-/tmp}/sesame-build"

case "$which" in
  main)   name=sesame-firmware-main; files=(firmware/sesame-firmware-main.ino firmware/*.h) ;;
  tester) name=sesame-motor-tester;  files=(firmware/debugging-firmware/sesame-motor-tester.ino) ;;
  *) echo "usage: $0 main|tester [port]"; exit 1 ;;
esac

rm -rf "$work/$name" && mkdir -p "$work/$name"
cp "${files[@]}" "$work/$name/"
arduino-cli compile --fqbn "$fqbn" --build-path "$work/$name-build" "$work/$name"
if [ -n "$port" ]; then
  arduino-cli upload --fqbn "$fqbn" --input-dir "$work/$name-build" -p "$port" "$work/$name"
fi
