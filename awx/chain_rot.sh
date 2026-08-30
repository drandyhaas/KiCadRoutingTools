#!/bin/bash
# GENERALITY GATE for the chain: the same checkpoints chained on a bench
# rotated by 90/180/270 degrees must grade the same (open, DRC, vias) as
# the unrotated board. A rotation is an isometry, so anything that
# changes is a stage leaning on the board's axes -- the plan's face
# names, the braid's flow frame, or a connection that assumed a
# direction. The rotated benches come from rotate_board.py (the flow
# frame proof made them); the chain is chain_k.sh with BASE set.
#
# usage: chain_rot.sh K [K...] [-- fanout options]
cd "$(dirname "$0")"
KS=()
while [ $# -gt 0 ] && [ "$1" != "--" ]; do KS+=("$1"); shift; done
[ "$1" = "--" ] && shift
for DEG in 90 180 270; do
  B="fb_rot${DEG}.kicad_pcb"
  if [ ! -f "$B" ]; then
    python3 rotate_board.py fb_t2q_base.kicad_pcb "$B" "$DEG" || exit 2
  fi
  echo "##### rotation $DEG"
  BASE="$B" bash chain_k.sh "crot${DEG}" "${KS[@]}" -- "$@" 2>&1 \
    | grep -E "^===|GRADE|NO |fanout board|WARNING: [A-Z]|kept floor"
done
echo "=== rotation gate done $(date +%H:%M:%S)"
