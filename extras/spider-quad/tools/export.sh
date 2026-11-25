#!/usr/bin/env bash
# Rebuild every printable part and the preview renders: bash spider-quad/tools/export.sh
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p stl renders

parts=(body_bottom body_top hip_bracket hip_bracket_mirror leg head)
log=$(mktemp)
for p in "${parts[@]}"; do
  openscad -o "stl/$p.stl" -D "part=\"$p\"" quad.scad >"$log" 2>&1 || { cat "$log"; exit 1; }
  if grep -E "WARNING|ERROR" "$log"; then exit 1; fi
  size=$(wc -c <"stl/$p.stl")
  [ "$size" -gt 1024 ] || { echo "stl/$p.stl is empty"; exit 1; }
  echo "stl/$p.stl  $size bytes"
done

cam="--colorscheme=Tomorrow --imgsize=1200,900"
openscad --render $cam --camera=0,0,20,62,0,35,520 -o renders/iso.png quad.scad >"$log" 2>&1
openscad --render $cam --camera=0,0,20,62,0,35,520 -D hip_pose=20 -D knee_pose=35 -o renders/walk.png quad.scad >"$log" 2>&1
openscad --render $cam --camera=0,0,20,90,0,90,420 --projection=o -o renders/side.png quad.scad >"$log" 2>&1
openscad --render $cam --camera=0,0,0,0,0,0,460 --projection=o -o renders/top.png quad.scad >"$log" 2>&1
grep "ECHO" "$log" || true
rm -f "$log"
echo "renders: $(ls renders | tr '\n' ' ')"
