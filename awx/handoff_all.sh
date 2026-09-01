#!/bin/bash
# Handoff render for each checkpoint into ~/Downloads/bus: the corridor
# leg the braid must draw, the escape it must join, and orange marks
# wherever that join does not work.
cd "$(dirname "$0")"
DEST=~/Downloads/bus
mkdir -p "$DEST"
for K in "$@"; do
  echo "=== K$K"
  python3 make_handoff.py "tmp/ho_k${K}.kicad_pcb" "$K" 2>&1 | tail -1
  VIEW=$(python3 view_of.py "$K")
  python3 render_eco.py "tmp/ho_k${K}.kicad_pcb" \
    "$DEST/take4_k${K}_handoff.png" \
    --nets "$(python3 coherent_nets.py "$K")" --view "$VIEW" 2>&1 \
    | sed 's/highlighted.*//'
  cp "tmp/ho_k${K}.kicad_pcb" "$DEST/take4_k${K}_handoff.kicad_pcb"
  [ -f "tmp/tmp/ho_k${K}.kicad_pro" ] && cp "tmp/tmp/ho_k${K}.kicad_pro" \
    "$DEST/take4_k${K}_handoff.kicad_pro"
done
echo "=== handoff renders done"
