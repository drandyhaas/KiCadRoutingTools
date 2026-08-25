#!/bin/bash
# Normalize+strip every set25 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set25/
#   normalized routed reference + .kicad_pro -> boards_set25/
# Mirrors prep_set10.sh. Sources are staged by fetch_set25.py into
# sources/github_set25/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set25"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set25" "$STRESS/boards_set25"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "peaksat_obc_adcs"
  "spartan6_4layer"
  "icebreaker_v10e"
  "lwdo_sdr"
  "muons_sipm_v2"
  "pd_electrode_v7"
  "espresso_hv"
  "ice4pi"
  "bms_sensor"
  "hv_pulse_gen"
  "sipm_bias_lt8362"
  "jqiamo_20a_front"
  "nesora_mixer"
  "opengammakit"
  "laser_backplane_dvi"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set25/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set25/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set25/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set25/ ; routed reference -> boards_set25/"
