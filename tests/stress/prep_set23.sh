#!/bin/bash
# Normalize+strip every set23 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set23/
#   normalized routed reference + .kicad_pro -> boards_set23/
# Mirrors prep_set10.sh. Sources are staged by fetch_set23.py into
# sources/github_set23/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set23"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set23" "$STRESS/boards_set23"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "mgh80_z80_sbc"
  "overlord_cm5"
  "azukar_fpga"
  "lm399_burnin"
  "healthypi_move"
  "miniscope_v4_flex"
  "micro_dmm"
  "apple2e_mmu"
  "cherrycam"
  "nascom_eprom_80bus"
  "klein_kb"
  "usb_dali"
  "cananka_fec"
  "a7s_backplane"
  "sipmcalib_hvlv"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set23/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set23/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set23/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set23/ ; routed reference -> boards_set23/"
