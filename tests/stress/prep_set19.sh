#!/bin/bash
# Normalize+strip every set19 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set19/
#   normalized routed reference + .kicad_pro -> boards_set19/
# Mirrors prep_set10.sh. Sources are staged by fetch_set19.py into
# sources/github_set19/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set19"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set19" "$STRESS/boards_set19"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "apple1_mainboard"
  "endgame_trackball"
  "home_io_board"
  "core64_logic"
  "macrolev"
  "nes_rom_vomitter"
  "z80_atmega128"
  "uni_orthosteno"
  "psycho_badge"
  "f3_nvme_backplane"
  "usp_obc_v7"
  "apple1_aci_card"
  "spwm_sine_inverter"
  "molekula_enc"
  "knx_reg1_rs485"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set19/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set19/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set19/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set19/ ; routed reference -> boards_set19/"
