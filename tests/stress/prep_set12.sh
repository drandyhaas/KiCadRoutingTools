#!/bin/bash
# Normalize+strip every set12 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set12/
#   normalized routed reference + .kicad_pro -> boards_set12/
# Mirrors prep_set10.sh. Sources are staged by fetch_set12.py into
# sources/github_set12/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set12"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set12" "$STRESS/boards_set12"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "allwinner_a13_som"
  "berkeley_obsidian"
  "lora_ice40_dongle"
  "duodyne_backplane20"
  "z80_devboard_rp2040"
  "abn6502"
  "pico_ice_rev3"
  "odmr_adf_board"
  "tallytime_lora"
  "bldc_tester"
  "kernelcon_badge"
  "tensorrail_mini"
  "myfocuserpro"
  "dlr_sensor_hat"
  "kiku_mesh"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set12/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set12/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set12/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set12/ ; routed reference -> boards_set12/"
