#!/bin/bash
# Normalize+strip every set22 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set22/
#   normalized routed reference + .kicad_pro -> boards_set22/
# Mirrors prep_set10.sh. Sources are staged by fetch_set22.py into
# sources/github_set22/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set22"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set22" "$STRESS/boards_set22"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "void_switch_65"
  "faderbank_16nx"
  "peaksat_comms"
  "crazyflie_fpga_deck"
  "usmu_smu"
  "astrouniboard"
  "esp32s3_m2_som"
  "reaction_wheel_foc"
  "daisy_pedal_board"
  "wisweep_driver"
  "om_flexgrid_v2_rigid"
  "nixie_in12_disp"
  "gpio_backplane_658"
  "openjoint"
  "bancouver40"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set22/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set22/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set22/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set22/ ; routed reference -> boards_set22/"
