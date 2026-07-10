#!/bin/bash
# Normalize+strip every set-9 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set9/
#   normalized routed reference + .kicad_pro -> boards_set9/
# Set 9: RF/wireless/networking (LoRa/BLE/Zigbee nodes, an SDR, a ham-radio
# handheld) + power electronics (USB-PD chargers, PoE injector, solar node,
# multi-rail robotics PSU). Sources are modern KiCad (v6+).
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set9"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set9" "$STRESS/boards_set9"

# short-name | source-filename-fragment (unique within github_set9/)
MAP=(
  "ble02_rf_module|mlabmodules__ble02"
  "zigbee_sensor|zektopic__zigbee_universal_sensor"
  "nrf51_ble_module|siderakb__nrf51spi-c"
  "stm32_rfm95_lora|tomtor__stm-lora"
  "cc1101_rf_module|jp112sdl__cc1101_module"
  "hex_gateway|gblach__hexgateway"
  "usbc_liion_charger|mg5__usb-c-to-liion-charger"
  "dc_poe_injector|jwpleow__dc_poe_injector"
  "fusb302_pd_dev|bentwire__usb-c-fusb302-pi"
  "minisolarmesh|aresta__minisolarmesh"
  "mokya_lora_phone|tengigabytes__mokyalora"
  "robotics_psu|thoric16__psu"
  "openairscope|emertcakir__openairscope"
  "linht_radio|m17project__linht-hw"
  "meshtastic_solar|h0lad__meshtastic_solar"
)

for entry in "${MAP[@]}"; do
  IFS='|' read -r name frag <<< "$entry"
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${frag}*" | head -1)
  if [ -z "$srcfile" ]; then echo "MISS $name (frag '$frag')"; continue; fi
  echo "== $name <- $(basename "$srcfile")"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set9/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set9/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="${srcfile%.kicad_pcb}.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set9/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set9/ ; routed reference -> boards_set9/"
