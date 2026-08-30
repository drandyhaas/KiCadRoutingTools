#!/bin/bash
# BASELINE: the pre-corridor braid (46a268b3's braid.py, extracted with
# git show into the untracked braid_head.py) on the same fanout boards
# ladder_n.sh uses, graded the same way.
# usage: ladder_head.sh TAG K [K...]   (FO=prefix, OUTDIR=where, REF=commit)
cd "$(dirname "$0")"
TAG=$1
shift
FO=${FO:-ch}
OUTDIR=${OUTDIR:-.}
REF=${REF:-46a268b3}
if [ ! -f braid_head.py ]; then
  git show "$REF:awx/braid.py" > braid_head.py || exit 2
fi
for K in "$@"; do
  echo "=== K$K $(date +%H:%M:%S)"
  NETS=$(python3 coherent_nets.py "$K")
  python3 -u braid_head.py --board "${FO}_fo_k${K}.kicad_pcb" \
    --dest "${DEST:-DU1}" --nets "$NETS" --out "$OUTDIR/${TAG}_k${K}" \
    > "$OUTDIR/${TAG}_k${K}.log" 2>&1
  grep -E "attempt|lanes:|flank corridor:|REFUSED|wrote" \
    "$OUTDIR/${TAG}_k${K}.log" | cut -c1-160
  if [ -f "$OUTDIR/${TAG}_k${K}.kicad_pcb" ]; then
    python3 grade_k.py "$OUTDIR/${TAG}_k${K}.kicad_pcb" "$NETS"
  else
    echo "NO BRAID"
    grep -E 'Error|assert|Traceback' -A4 "$OUTDIR/${TAG}_k${K}.log" | tail -8
  fi
done
echo "=== baseline ladder done $(date +%H:%M:%S)"
