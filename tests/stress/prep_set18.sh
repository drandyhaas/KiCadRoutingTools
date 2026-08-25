#!/bin/bash
# Normalize+strip every set18 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set18/
#   normalized routed reference + .kicad_pro -> boards_set18/
# Mirrors prep_set10.sh. Sources are staged by fetch_set18.py into
# sources/github_set18/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set18"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set18" "$STRESS/boards_set18"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "libresolar_bms_c1"
  "fsae_csc"
  "hackgdl_badge"
  "argus_solarpanel"
  "moteus_c1"
  "pico_2x_bldc"
  "openinverter_mini"
  "fsae_precharge"
  "eez_aux_ps"
  "arctyx_nano"
  "chart_plotter_hat"
  "tmc2209_dev"
  "rp2040_trackball"
  "quad_rs485_pico"
  "dog_harness_panel"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set18/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set18/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set18/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set18/ ; routed reference -> boards_set18/"
