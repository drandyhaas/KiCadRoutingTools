#!/bin/bash
# Normalize+strip every set-3 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set3/
#   normalized routed reference + .kicad_pro -> boards_set3/
# Many set-3 sources are pre-v6 KiCad (v4 / v20171130); LoadBoard upgrades them.
# Run AFTER set 1/2 so big-board pcbnew loads don't OOM a small box.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set3"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set3" "$STRESS/boards_set3"

# short-name | source-filename-fragment (unique within github_set3/)
MAP=(
  "lora_v3|strooom__LoRa-V3"
  "daisho|greatscottgadgets__daisho__"
  "orangecrab|orangecrab-fpga__"
  "keks|machdyne__keks__"
  "eis|machdyne__eis__"
  "upduino|tinyvision-ai-inc__"
  "tigard|tigard-tools__"
  "watchy|sqfmi__watchy"
  "lna3030|greatscottgadgets__LNA3030"
  "fomu|im-tomu__fomu"
  "olimex_lora|OLIMEX__LoRa-868"
  "a20_can|OLIMEX__A20-CAN"
  "esp_prog|OLIMEX__ESP-PROG"
  "mod_bme280|OLIMEX__MOD-BME280"
  "throwing_star|greatscottgadgets__throwing-star"
)

for entry in "${MAP[@]}"; do
  IFS='|' read -r name frag <<< "$entry"
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${frag}*" | head -1)
  if [ -z "$srcfile" ]; then echo "MISS $name (frag '$frag')"; continue; fi
  echo "== $name <- $(basename "$srcfile")"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set3/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set3/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="${srcfile%.kicad_pcb}.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set3/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set3/ ; routed reference -> boards_set3/"
