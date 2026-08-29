#!/bin/bash
# Prove the FLOW FRAME: the same checkpoints routed on a bench rotated
# by 90/180/270 degrees must come out with the same via count and the
# same clean grade as the unrotated board. A rotation is an isometry, so
# anything that changes is the router leaning on the board's axes.
cd "$(dirname "$0")"
for DEG in 90 180 270; do
  B="fb_rot${DEG}.kicad_pcb"
  if [ ! -f "$B" ]; then
    python3 rotate_board.py fb_t2q_base.kicad_pcb "$B" "$DEG" || exit 2
  fi
  for K in "$@"; do
    NETS=$(python3 coherent_nets.py "$K")
    python3 -u topo_emit.py --board "$B" --nets "$NETS" \
      --out "rot${DEG}_k${K}" > "rot${DEG}_k${K}.log" 2>&1
    if [ ! -f "rot${DEG}_k${K}.kicad_pcb" ]; then
      echo "rot${DEG} K$K  NO BOARD: $(grep -E 'Error|assert' "rot${DEG}_k${K}.log" | tail -1)"
      continue
    fi
    echo -n "rot${DEG} "
    python3 grade_k.py "rot${DEG}_k${K}.kicad_pcb" "$NETS"
  done
done
echo "=== rotation proof done"
