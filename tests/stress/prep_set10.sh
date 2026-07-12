#!/bin/bash
# Normalize+strip every set-10 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set10/
#   normalized routed reference + .kicad_pro -> boards_set10/
# Mirrors prep_set5.sh. Sources are modern KiCad (v6+, kicad_version >= 20211014).
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set10"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set10" "$STRESS/boards_set10"

# short-name | source-filename-fragment (unique within github_set10/)
MAP=(
  "cryologger_aws|cryologger__aws_v0.4"
  "unl_flight_computer|unlrocketry__flight_computer_core"
  "picoprobe_tester|nevynuk__picoprobe_tester"
  "nano_eeprom_prog|cgmatt__nano_eeprom_programmer"
  "esp32cam_pcb|unsignedarduino__esp32_camera_pcb"
  "orbtrace_bob|orbcode__orbtrace_bob2"
  "stm32g474_fc|def12345678__m_fc_g474"
  "noahfc|zosko__noahfc"
  "steppenprobe|diegoherranz__steppenprobe"
  "open_weather_station|vitalseptfonds__openweatherstation_mainboard"
  "bus_pirate5|dangerousprototypes__buspirate5_rev10a"
  "duet2_3dp|duet3d__duet2_v106"
  "sata_sniffer|azonenberg__sata_sniffer"
  "starlighteye_cam|will127534__starlighteye"
  "eth_tap|azonenberg__ethernet_tap"
)

for entry in "${MAP[@]}"; do
  IFS='|' read -r name frag <<< "$entry"
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${frag}*" | head -1)
  if [ -z "$srcfile" ]; then echo "MISS $name (frag '$frag')"; continue; fi
  echo "== $name <- $(basename "$srcfile")"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set10/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set10/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="${srcfile%.kicad_pcb}.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set10/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set10/ ; routed reference -> boards_set10/"
