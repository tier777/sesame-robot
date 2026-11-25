#!/usr/bin/env bash
# Пересобрать STL и картинки: bash hardware/battery-mount/tools/export.sh
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p stl renders
log=$(mktemp)

# седло не должно врезаться в верхнюю крышку
openscad -o "$log.stl" -D 'part="check"' battery-mount.scad >"$log" 2>&1 || true
grep -q "Current top level object is empty" "$log" || { cat "$log"; echo "седло пересекается с крышкой"; exit 1; }

for p in saddle tray; do
  openscad --export-format binstl -o "stl/$p.stl" -D "part=\"$p\"" battery-mount.scad >"$log" 2>&1 || { cat "$log"; exit 1; }
  if grep -E "WARNING|ERROR" "$log"; then exit 1; fi
  echo "stl/$p.stl  $(wc -c <"stl/$p.stl") bytes"
done

cam="--colorscheme=Tomorrow --imgsize=1200,900 --render"
openscad $cam --camera=26,0,40,60,0,215,260 -o renders/iso.png battery-mount.scad >"$log" 2>&1
openscad $cam --camera=26,0,45,90,0,0,280 --projection=o -o renders/side.png battery-mount.scad >"$log" 2>&1
openscad $cam --camera=26,0,45,90,0,90,240 --projection=o -o renders/back.png battery-mount.scad >"$log" 2>&1
openscad $cam --camera=26,0,10,50,0,25,260 -D 'part="print"' -o renders/print.png battery-mount.scad >"$log" 2>&1
grep "ECHO" "$log" || true
rm -f "$log" "$log.stl"
echo "renders: $(ls renders | tr '\n' ' ')"
