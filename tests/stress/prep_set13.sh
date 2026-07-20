#!/bin/bash
# Normalize+strip every set13 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set13/
#   normalized routed reference + .kicad_pro -> boards_set13/
# Mirrors prep_set10.sh. Sources are staged by fetch_set13.py into
# sources/github_set13/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set13"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set13" "$STRESS/boards_set13"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "cubesat_backplane"
  "zynthian_v5_control"
  "anyradio_cm5"
  "sband_transceiver"
  "pocketsdr_4ch"
  "qfhmix01"
  "sata_backplane"
  "len42_fx"
  "rio_iceshield"
  "nhyodyne_6809"
  "g473_gen1"
  "kawaii_dock"
  "pt2399_delay"
  "openmux_potentio"
  "tiny_vga_pmod"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set13/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set13/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set13/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set13/ ; routed reference -> boards_set13/"
