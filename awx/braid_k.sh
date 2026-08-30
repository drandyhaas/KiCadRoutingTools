#!/bin/bash
# Run the braid on an existing fanout board for one K, grade it, and
# print the log's summary lines. Output boards go to $OUTDIR (default:
# the scratchpad if set, else here).
#
# usage: braid_k.sh FO_BOARD TAG K
cd "$(dirname "$0")"
FO=$1
TAG=$2
K=$3
OUTDIR=${OUTDIR:-.}
NETS=$(python3 coherent_nets.py "$K")
python3 -u braid.py --board "$FO" --dest "${DEST:-DU1}" --nets "$NETS" \
  --out "$OUTDIR/${TAG}_k${K}" > "$OUTDIR/${TAG}_k${K}.log" 2>&1
RC=$?
grep -E "corridor\(s\)|^corridor |spine:|s0=|joiners:|side exits:|reserved:|attempt|lanes:|refused|REFUSED|smooth_oct|wrote|Traceback|Error|assert" \
  "$OUTDIR/${TAG}_k${K}.log" | cut -c1-220
if [ -f "$OUTDIR/${TAG}_k${K}.kicad_pcb" ]; then
  python3 grade_k.py "$OUTDIR/${TAG}_k${K}.kicad_pcb" "$NETS"
else
  echo "NO BRAID (rc=$RC):"
  grep -E 'Error|assert|Traceback' -A6 "$OUTDIR/${TAG}_k${K}.log" | tail -12
fi
