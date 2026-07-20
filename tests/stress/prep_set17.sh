#!/bin/bash
# Normalize+strip every set17 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set17/
#   normalized routed reference + .kicad_pro -> boards_set17/
# Mirrors prep_set10.sh. Sources are staged by fetch_set17.py into
# sources/github_set17/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set17"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set17" "$STRESS/boards_set17"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "key_ripper"
  "moteus_x1"
  "tesla_coil_sensor"
  "artix_devboard"
  "epdiy_kaleido"
  "keychron_optical"
  "tk44"
  "sensorwatch_round"
  "esp32_iot_sd"
  "punck_components"
  "rfswitch01"
  "ploopy_nano"
  "hexapod_legbrd"
  "triad_actuator"
  "in13_nixie"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set17/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set17/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set17/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set17/ ; routed reference -> boards_set17/"
