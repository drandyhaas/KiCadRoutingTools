#!/bin/bash
# Normalize+strip every set24 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set24/
#   normalized routed reference + .kicad_pro -> boards_set24/
# Mirrors prep_set10.sh. Sources are staged by fetch_set24.py into
# sources/github_set24/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set24"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set24" "$STRESS/boards_set24"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "granit_cm5"
  "din41612_backplane"
  "ykts_payload"
  "allwinner_h3_ddr3"
  "healthypi_sensor"
  "dawson_trigger"
  "bornhack_circle_badge"
  "stm32wl_lora_rf"
  "om_flexgrid_rigid"
  "pocat_comms"
  "sx1262_re_433"
  "rc2014_82c55_ide"
  "gb_nintendo_power"
  "galvoexpress"
  "pmw3360_mouse"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set24/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set24/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set24/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set24/ ; routed reference -> boards_set24/"
