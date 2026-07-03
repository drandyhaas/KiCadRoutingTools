#!/bin/bash
# Prep the EXTREME "set1monster" corpus (huge 6-8 layer / 400-900fp / up-to-62MB
# open-hardware boards). One pcbnew process per board (segfault-safe); these are
# SLOW and memory-heavy -- run them one at a time, not in parallel.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set1monster"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set1monster" "$STRESS/boards_set1monster"
MAP=(
  "thunderscope|eevengers__thunderscope_r53"
  "comexpress7|antmicro__comexpress7"
  "kria_k26|antmicro__kria_k26"
  "snapdragon845|antmicro__snapdragon845"
  "hackrf_pro|gsg__hackrf_pro"
  "artix_dc_scm|antmicro__artix_dc_scm"
  "urti|gsg__urti_mainboard"
)
for entry in "${MAP[@]}"; do
  IFS='|' read -r name frag <<< "$entry"
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${frag}*" | head -1)
  [ -z "$srcfile" ] && { echo "MISS $name"; continue; }
  echo "== $name <- $(basename "$srcfile") ($(ls -lh "$srcfile"|awk '{print $5}'))  $(date '+%H:%M:%S')"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set1monster/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set1monster/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="${srcfile%.kicad_pcb}.kicad_pro"; [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set1monster/$name.kicad_pro"
done
echo "Done set1monster."
