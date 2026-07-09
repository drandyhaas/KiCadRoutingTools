#!/bin/bash
# Normalize+strip every set-6 board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_set6/
#   normalized routed reference + .kicad_pro -> boards_set6/
# Set 6 theme: FPGA/CPLD development boards + embedded-Linux SBC carrier
# boards (RISC-V MCU dev boards, Lattice/Xilinx/CologneChip FPGA boards,
# Jetson Nano / CM4 / Zynq SoM carriers). Sources are modern KiCad (v6+).
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STRESS="${STRESS_DIR:-$HOME/Documents/kicad_stress_test}"
SRC="$STRESS/sources/github_set6"
KPY="${KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_set6" "$STRESS/boards_set6"

# short-name | source-filename-fragment (unique within github_set6/)
MAP=(
  "ch32v006_dev|ch32v006_dev"
  "ch32v303_dev|ch32v303_dev"
  "ch32v003_usb|ch32v003_usbdev"
  "ice5lp4k_hat|ice5lp4k_hat"
  "atf1502_cpld|atf1502_evb"
  "qmtech_k7|qmtech_k7"
  "tsuraragb|tsuraragb"
  "quickfeather|quickfeather"
  "mdfpga_hub75|mdfpga"
  "ch32v203_ev|ch32v203_ev"
  "microzed_carrier|microzed_carrier"
  "scalenode_cm4|scalenode_cm4"
  "zynq7020_som|zynq_som2"
  "hexberry_fpga|hexberry"
  "ulx5m_gatemate|ulx5m_gs"
)

for entry in "${MAP[@]}"; do
  IFS='|' read -r name frag <<< "$entry"
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${frag}*" | head -1)
  if [ -z "$srcfile" ]; then echo "MISS $name (frag '$frag')"; continue; fi
  echo "== $name <- $(basename "$srcfile")"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_set6/$name.kicad_pcb" \
         "$STRESS/boards_unrouted_set6/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="${srcfile%.kicad_pcb}.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_set6/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_set6/ ; routed reference -> boards_set6/"
