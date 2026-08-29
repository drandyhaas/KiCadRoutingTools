#!/bin/bash
# Intent overlay + render for every checkpoint, into ~/Downloads/bus.
# The view is computed per K from that checkpoint's own nets plus the
# destination array, so a large K is not cropped to a small K's window.
cd "$(dirname "$0")"
DEST=~/Downloads/bus
mkdir -p "$DEST"
for K in "$@"; do
  echo "=== K$K  $(date +%H:%M:%S)"
  python3 make_overlay.py "ov_k${K}.kicad_pcb" "$K" 2>&1 | tail -2
  VIEW=$(python3 view_of.py "$K")
  python3 render_eco.py "ov_k${K}.kicad_pcb" "$DEST/take4_k${K}_intent.png" \
    --nets "$(python3 coherent_nets.py "$K")" --view "$VIEW" 2>&1 | tail -1
  cp "ov_k${K}.kicad_pcb" "$DEST/take4_k${K}_intent.kicad_pcb"
  [ -f ov_k${K}.kicad_pro ] && cp "ov_k${K}.kicad_pro" \
    "$DEST/take4_k${K}_intent.kicad_pro"
done
echo "=== renders done $(date +%H:%M:%S)"
