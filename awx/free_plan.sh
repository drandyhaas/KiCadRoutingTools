#!/bin/bash
# The plan WITHOUT the --dirs face restriction, at the given Ks: what
# faces does it choose when every face is on offer? Output goes to
# $OUTDIR (default: here), one fanout board + log per K.
#
# usage: free_plan.sh K [K...]
cd "$(dirname "$0")"
OUTDIR=${OUTDIR:-.}
for K in "$@"; do
  python3 fanout_from_plan.py "$OUTDIR/free_fo_k${K}.kicad_pcb" "$K" \
    --no-lines --no-plane-drop > "$OUTDIR/free_fo_k${K}.log" 2>&1
  echo "=== K$K"
  grep -E "kept floor|^plan:|obeyed|failed nets|unplaced" \
    "$OUTDIR/free_fo_k${K}.log"
done
echo "=== free plans done"
