#!/bin/bash
# The fanout stage over the ladder, twice: the plan RESTRICTED to the
# faces the old braid could deliver to (r_fo_k*, --dirs=left,down) and
# the plan FREE to use every face (f_fo_k*). Both with --no-lines and
# --no-plane-drop (the README ladder's terms). Boards go to $OUTDIR.
# usage: fanout_ladder.sh K [K...]
cd "$(dirname "$0")"
OUTDIR=${OUTDIR:-.}
for K in "$@"; do
  for ARM in r f; do
    if [ "$ARM" = r ]; then D="--dirs=left,down"; else D=""; fi
    python3 fanout_from_plan.py "$OUTDIR/${ARM}_fo_k${K}.kicad_pcb" "$K" \
      $D --no-lines --no-plane-drop > "$OUTDIR/${ARM}_fo_k${K}.log" 2>&1
    echo "=== ${ARM} K$K: $(grep -E '^plan:' "$OUTDIR/${ARM}_fo_k${K}.log")  $(grep -E 'obeyed' "$OUTDIR/${ARM}_fo_k${K}.log")"
    python3 ../py_router/check_drc.py "$OUTDIR/${ARM}_fo_k${K}.kicad_pcb" \
      --clearance 0.1 --clearance-margin 0.1 2>&1 | grep -E "FOUND|NO DRC" \
      | sed 's/^/    fanout board: /'
  done
done
echo "=== fanout ladder done $(date +%H:%M:%S)"
