#!/bin/bash
# NEGATIVE CONTROL for the flow frame: the same rotated boards with the
# frame DISABLED. If these also pass, the frame is doing nothing and the
# rotation proof is vacuous.
cd "$(dirname "$0")"
for DEG in 90 180 270; do
  B="fb_rot${DEG}.kicad_pcb"
  for K in "$@"; do
    NETS=$(python3 coherent_nets.py "$K")
    KICAD_FLOW_FRAME=0 python3 -u topo_emit.py --board "$B" \
      --nets "$NETS" --out "ctl${DEG}_k${K}" > "ctl${DEG}_k${K}.log" 2>&1
    if [ ! -f "ctl${DEG}_k${K}.kicad_pcb" ]; then
      echo "rot${DEG} K$K  NO BOARD: $(grep -E 'Error|assert' "ctl${DEG}_k${K}.log" | tail -1 | cut -c1-90)"
      continue
    fi
    echo -n "rot${DEG} NOFRAME "
    python3 grade_k.py "ctl${DEG}_k${K}.kicad_pcb" "$NETS"
  done
done
echo "=== control done"
