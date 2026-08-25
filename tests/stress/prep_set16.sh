#!/bin/bash
# Normalize+strip every set16 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set16/
#   normalized routed reference + .kicad_pro -> boards_set16/
# Mirrors prep_set10.sh. Sources are staged by fetch_set16.py into
# sources/github_set16/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set16"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set16" "$STRESS/boards_set16"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "ethercat_ax58100"
  "argus_mainboard"
  "spartan6_6layer"
  "music32_v3"
  "teacup_carrier"
  "ecp5_sbc_mobo"
  "ethercat_8in8out"
  "eez_dib_b3c"
  "icev_wireless"
  "adc_octo_spi"
  "byobc_6502_debugger"
  "fire_panel_rs485"
  "fidget_spinner_imu"
  "dmx_control_board"
  "spimux"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set16/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set16/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set16/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set16/ ; routed reference -> boards_set16/"
