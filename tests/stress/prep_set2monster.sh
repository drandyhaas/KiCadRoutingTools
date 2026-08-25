#!/bin/bash
# Prep the EXTREME "set2monster" corpus (huge 4-8 layer / 110-680fp / up-to-37MB
# open-hardware boards: AI-accelerator/compute-carrier baseboards, server/telecom
# adapters, a GPU cluster backplane, an LPDDR5 testbed, camera/broadcast boards).
# One pcbnew process per board (segfault-safe); these are SLOW and memory-heavy --
# run them one at a time, not in parallel.
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set2monster"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set2monster" "$STRESS/boards_set2monster"
MAP=(
  "poe_usbc_pd|antmicro__poe_usbc_pd"
  "cm4_lvds|antmicro__cm4_lvds_adapter"
  "snapdragon625|antmicro__snapdragon625_baseboard"
  "hdmi_mipi_bridge|antmicro__hdmi_mipi_bridge"
  "oculink_10gbe|antmicro__oculink_10gbe_adapter"
  "sdi_mipi_conv|antmicro__sdi_mipi_video_converter"
  "ddr4_spd_breakout|antmicro__ddr4_spd_breakout"
  "lpddr5_testbed|antmicro__lpddr5_testbed"
  "sdi_mipi_bridge|antmicro__sdi_mipi_bridge"
  "oculink_pcie|antmicro__oculink_pcie_adapter"
  "m2_iot|antmicro__m2_smart_iot_module"
  "ovc5_camera_carrier|osrf__ovc5_camera_carrier"
  "gem_baseboard|antmicro__gem_baseboard"
  "agx_thor_interposer|antmicro__agx_thor_interposer"
  "gmsl_serializer|antmicro__gmsl_serializer_adapter"
)
for entry in "${MAP[@]}"; do
  IFS='|' read -r name frag <<< "$entry"
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${frag}*" | head -1)
  [ -z "$srcfile" ] && { echo "MISS $name"; continue; }
  echo "== $name <- $(basename "$srcfile") ($(ls -lh "$srcfile"|awk '{print $5}'))  $(date '+%H:%M:%S')"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set2monster/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set2monster/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="${srcfile%.kicad_pcb}.kicad_pro"; [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set2monster/$name.kicad_pro"
done
echo "Done set2monster."
