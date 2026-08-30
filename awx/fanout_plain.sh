#!/bin/bash
# Fan out the destination with the PRODUCTION bga_fanout and no plan
# hints -- for benches the plan's grid menus cannot read (a rotated
# array: escape_moves measures rows and columns in board axes), so the
# braid can still be exercised on them. Same geometry as the chain's
# fanout step (track 0.1, clearance 0.1, via 0.45/0.25, exit margin
# 0.5, plane-drop off), graded on its own.
# usage: fanout_plain.sh BASE OUT K [DEST]
cd "$(dirname "$0")"
BASE=$1
OUT=$2
K=$3
DEST=${4:-DU1}
NETS=$(python3 coherent_nets.py "$K" | tr ',' ' ')
python3 ../py_router/bga_fanout.py "$BASE" -o "$OUT.kicad_pcb" -c "$DEST" \
  -n $NETS -l F.Cu B.Cu -w 0.1 --clearance 0.1 --via-size 0.45 \
  --via-drill 0.25 --exit-margin 0.5 --plane-drop off > "$OUT.log" 2>&1
grep -E "JSON_SUMMARY|escaped|failed|WARNING" "$OUT.log" | cut -c1-200 | tail -4
PRO=${BASE%.kicad_pcb}.kicad_pro
[ -f "$PRO" ] && cp "$PRO" "$OUT.kicad_pro"
bash drc_of.sh "$OUT.kicad_pcb" 4
