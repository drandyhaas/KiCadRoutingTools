#!/bin/bash
# Normalize+strip every set-8 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set8/
#   normalized routed reference + .kicad_pro -> boards_set8/
# Set 8 theme: mechanical keyboards/HID input devices + sensor/instrumentation
# boards. Sources are modern KiCad (v6+).
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set8"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set8" "$STRESS/boards_set8"

# short-name | source-filename-fragment (unique within github_set8/)
MAP=(
  "piantor|beekeeb__piantor_left"
  "pinci|camrbuss__pinci"
  "uhk_trackball|uhk__trackball"
  "co2_monitor|miekush__co2_monitor"
  "rp2040_numpad|parkergreene__numpad"
  "corax56|dnlbauer__corax56"
  "mantis_kb|fxkuehl__mantis"
  "hhkb_ctrl|keebio__hhkb_controller"
  "vis_nir_spec|nviale__embedded_board"
  "chocofi|pashutk__chocofi"
  "kbic65|bkarl__kbic65"
  "urchin_kb|duckyb__urchin"
  "bfo9000|keebio__bfo9000"
  "orbiter_kb|kjeller__orbiter"
  "weatherstation|ppusapati__weatherstation_mfg"
)

for entry in "${MAP[@]}"; do
  IFS='|' read -r name frag <<< "$entry"
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${frag}*" | head -1)
  if [ -z "$srcfile" ]; then echo "MISS $name (frag '$frag')"; continue; fi
  echo "== $name <- $(basename "$srcfile")"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set8/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set8/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="${srcfile%.kicad_pcb}.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set8/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set8/ ; routed reference -> boards_set8/"
