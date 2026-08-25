#!/bin/bash
# Normalize+strip every set14 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set14/
#   normalized routed reference + .kicad_pro -> boards_set14/
# Mirrors prep_set10.sh. Sources are staged by fetch_set14.py into
# sources/github_set14/.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set14"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set14" "$STRESS/boards_set14"

# short-name (the staged source file is <short-name>.kicad_pcb)
NAMES=(
  "rusefi_alphax4"
  "kintex_pcie"
  "librevna"
  "xerxes_cm4_blade"
  "cm5_minima2"
  "cm4_underwater"
  "epdiy_v7"
  "antminer_zynq_hat"
  "mystat_potentiostat"
  "tx_band_splitter"
  "awkb_36_split"
  "robocup_kicker"
  "isa_option_rom"
  "mbira_esp_midi"
  "scram_ddrn"
)

for name in "${NAMES[@]}"; do
  srcfile="$SRC/$name.kicad_pcb"
  if [ ! -f "$srcfile" ]; then echo "MISS $name"; continue; fi
  echo "== $name"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set14/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set14/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="$SRC/$name.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set14/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set14/ ; routed reference -> boards_set14/"
