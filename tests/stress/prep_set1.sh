#!/bin/bash
# Normalize+strip every set1 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set1/
#   normalized routed reference + .kicad_pro -> boards_set1/
# Sources are staged by fetch_set1.py into
# sources/github_set1/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set1"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set1" "$STRESS/boards_set1"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "castor_pollux"
  "glasgow_revC"
  "hackrf_one"
  "haxophone"
  "lpddr4_testbed"
  "lumenpnp_mobo"
  "megadesk"
  "neo6502"
  "ottercast_audio"
  "piantor"
  "smartknob_base"
  "splitflap_driver"
  "upsy_desky"
  "urchin"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set1/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set1/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set1/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set1/ ; routed reference -> boards_set1/"
