#!/bin/bash
# Normalize+strip every set15 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set15/
#   normalized routed reference + .kicad_pro -> boards_set15/
# Mirrors prep_set10.sh. Sources are staged by fetch_set15.py into
# sources/github_set15/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set15"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set15" "$STRESS/boards_set15"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "tildagon_base"
  "pcie_test_edge"
  "retrospector_components"
  "covg_patchclamp"
  "kirdy_laser_drv"
  "anyshake_explorer"
  "tildagon_top"
  "reterminal_em"
  "kishoof_components"
  "loratracker_hat"
  "moco_bkd8316"
  "flipper_ethernet"
  "nixie_synthron"
  "gb_cart256k"
  "bm_svf"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set15/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set15/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set15/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set15/ ; routed reference -> boards_set15/"
