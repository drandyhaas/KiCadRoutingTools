#!/bin/bash
# Normalize+strip every set-11 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set11/
#   normalized routed reference + .kicad_pro -> boards_set11/
# Mirrors prep_set10.sh. Set 11 boards are staged by
# fetch_set11.py into sources/local_set11/ (zip-published sources, e.g.
# Adiuvo Engineering's forgix_public). rp2350_fpga_eensy's outline lives
# in a footprint; prep_set2.py's GetBoardPolygonOutlines rebuild emits it as
# board-level Edge.Cuts so the text parser sees board_bounds.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/local_set11"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set11" "$STRESS/boards_set11"

# short-name | source-filename-fragment (unique within local_set11/)
MAP=(
  "rp2350_fpga_eensy|RP2350_FPGA_eensy"
  "mikoto_nrf52840|mikoto"
)

for entry in "${MAP[@]}"; do
  IFS='|' read -r name frag <<< "$entry"
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${frag}*" | head -1)
  if [ -z "$srcfile" ]; then echo "MISS $name (frag '$frag')"; continue; fi
  echo "== $name <- $(basename "$srcfile")"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set11/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set11/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="${srcfile%.kicad_pcb}.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set11/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set11/ ; routed reference -> boards_set11/"
