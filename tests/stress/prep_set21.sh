#!/bin/bash
# Normalize+strip every set21 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set21/
#   normalized routed reference + .kicad_pro -> boards_set21/
# Mirrors prep_set10.sh. Sources are staged by fetch_set21.py into
# sources/github_set21/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set21"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set21" "$STRESS/boards_set21"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "muzy_zynq4"
  "muzy_zynq2"
  "cm4_edge_ai"
  "nanovoltmeter_marge"
  "varner_sipm_revb"
  "uncutgem_nv"
  "polykit_x_inputboard"
  "ember_he"
  "mydewcontroller"
  "picofx_pump"
  "kivu12"
  "wrass_audio_card"
  "magic_keys"
  "universal_badge_led"
  "led_ring_crossbar"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set21/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set21/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set21/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set21/ ; routed reference -> boards_set21/"
