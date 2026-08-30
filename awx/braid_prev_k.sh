#!/bin/bash
# The PRE-CHANGE braid (HEAD's braid.py extracted to braid_prev.py) on one
# fanout board, graded like braid_k.sh. usage: braid_prev_k.sh FO TAG K
cd "$(dirname "$0")"
FO=$1
TAG=$2
K=$3
OUTDIR=${OUTDIR:-.}
NETS=$(python3 coherent_nets.py "$K")
python3 -u braid_prev.py --board "$FO" --dest "${DEST:-DU1}" --nets "$NETS" \
  --out "$OUTDIR/${TAG}_k${K}" > "$OUTDIR/${TAG}_k${K}.log" 2>&1
grep -E "corridor\(s\)|attempt 0|lanes:|REFUSED lanes|wrote" "$OUTDIR/${TAG}_k${K}.log" | cut -c1-200
python3 grade_k.py "$OUTDIR/${TAG}_k${K}.kicad_pcb" "$NETS"
