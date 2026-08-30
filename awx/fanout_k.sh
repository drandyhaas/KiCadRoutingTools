#!/bin/bash
# The fanout stage of the chain alone, graded on its own: plan the
# berth's exit sides, fan the berth out in them, then DRC the fanout
# board BEFORE any braid touches it -- a fanout that ships stub-vs-stub
# contact (K15 with line hints: SDQM1/SDQ14 ended at one point) is a
# broken berth, and a braid graded on top of it measures the wrong thing.
#
# usage: fanout_k.sh OUT_TAG K [fanout_from_plan options...]
cd "$(dirname "$0")"
OUT=$1
K=$2
shift 2
NETS=$(python3 coherent_nets.py "$K")
python3 fanout_from_plan.py "$OUT.kicad_pcb" "$K" "$@" > "$OUT.log" 2>&1
grep -E "kept floor|^plan:|dropped|^wrote|obeyed|differed|failed nets" \
  "$OUT.log"
python3 ../py_router/check_drc.py "$OUT.kicad_pcb" --clearance 0.1 \
  --clearance-margin 0.1 2>&1 | grep -E "FOUND|<->" | sort | uniq -c \
  | sort -rn | head -8
python3 probe_chain_geom.py "$OUT.kicad_pcb" "$K" --dest DU1 \
  | tail -n +6
