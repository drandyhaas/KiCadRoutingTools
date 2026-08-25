#!/bin/bash
# Normalize+strip every set20 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set20/
#   normalized routed reference + .kicad_pro -> boards_set20/
# Mirrors prep_set10.sh. Sources are staged by fetch_set20.py into
# sources/github_set20/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set20"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set20" "$STRESS/boards_set20"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "pocketsdr_8ch"
  "fsae_bms_slave"
  "cb_static"
  "eternal_keypad"
  "fsae_busbar_6l"
  "epr_bridge_ctrl"
  "limeskey_mesh"
  "simplebldc_inv"
  "vigor_frontpanel"
  "diode_rom_matrix"
  "hifi_dsp_board"
  "cheapmesh"
  "divtiesus_zx"
  "vvvf_bridge"
  "mini_motor_ctrl"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set20/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set20/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set20/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set20/ ; routed reference -> boards_set20/"
