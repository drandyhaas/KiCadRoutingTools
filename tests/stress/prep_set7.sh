#!/bin/bash
# Normalize+strip every set-7 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set7/
#   normalized routed reference + .kicad_pro -> boards_set7/
# Set 7 theme: audio/synth electronics + robotics/motor-controller boards.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set7"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set7" "$STRESS/boards_set7"

# short-name | source-filename-fragment (unique within github_set7/)
MAP=(
  "rotten_pedal|kamikazevildsvin__rotten"
  "pedal_404|chrismettal__pedaldev__404"
  "drawbot_ctrl|tuckbick__drawbot_controller"
  "bread_dcmotor|feastorg__slice_dcmt"
  "bread_stepper|feastorg__stepper_card"
  "helium_mult|wntrblm__helium"
  "thatmicpre|ojg__thatmicpre_v2"
  "nudac|danchouzhou__nudac"
  "ethersweep|neumi__ethersweep405"
  "ackack_rover|aesilky__ackack_ctrlbd"
  "polykit_x|polykit__polykit-x-monosynth"
  "micro_ox_vco|jordanaceto__micro_ox_vco"
  "dsp_adau1452|ohdsp__dsp-adau1452"
  "rp2040_motorctrl|twistedfields__rp2040_base"
  "kevinbot_v3|meowmeowahr__kevinbotv3"
)

for entry in "${MAP[@]}"; do
  IFS='|' read -r name frag <<< "$entry"
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${frag}*" | head -1)
  if [ -z "$srcfile" ]; then echo "MISS $name (frag '$frag')"; continue; fi
  echo "== $name <- $(basename "$srcfile")"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set7/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set7/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="${srcfile%.kicad_pcb}.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set7/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set7/ ; routed reference -> boards_set7/"
