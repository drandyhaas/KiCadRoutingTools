#!/bin/bash
# Prep the "set3monster" EXTREME corpus (intractable / cap-exceeding boards).
# First member: lora_cubesat_cm -- 485 nets / 6 layers, the first corpus board
# whose single signal-route step exceeds the RUNBOOK 3h/command cap. One pcbnew
# process per board (segfault-safe); these are SLOW and memory-heavy.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set3monster"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set3monster" "$STRESS/boards_set3monster"
# short-name | source-filename-fragment (unique within github_set3monster/)
MAP=(
  "lora_cubesat_cm|lora_cubesat_cm"
)
for entry in "${MAP[@]}"; do
  IFS='|' read -r name frag <<< "$entry"
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${frag}*" | head -1)
  [ -z "$srcfile" ] && { echo "MISS $name"; continue; }
  echo "== $name <- $(basename "$srcfile") ($(ls -lh "$srcfile"|awk '{print $5}'))  $(date '+%H:%M:%S')"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set3monster/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set3monster/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="${srcfile%.kicad_pcb}.kicad_pro"; [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set3monster/$name.kicad_pro"
done
echo "Done set3monster."
