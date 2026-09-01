#!/bin/bash
# #622 solver ladder: per K -- solve, oracle fanout, chain both arms
cd "$(dirname "$0")"
for K in "$@"; do
  echo "===== K$K ====="
  python3 plan_global.py solve --k "$K" --out-prefix "solved_k$K" \
    2>&1 | grep -E 'SOLVED|PENDING|PINNED'
  python3 plan_fanout.py "$K" --contract-json "solved_k${K}_contract.json" \
    --out "plan_sv$K.kicad_pcb" > "plan_sv$K.log" 2>&1
  grep -E 'PLAN OBEDIENCE.*rescue|DRC after' "plan_sv$K.log" | sed 's/^/  /'
  TWO_PAGE=1 TP_SCOPE=split PLAN_OPTS="--two-page" \
    bash chain_k.sh "lc$K" "$K" -- --no-plane-drop 2>&1 | grep GRADE
  BASE="plan_sv$K.kicad_pcb" TWO_PAGE=1 TP_SCOPE=split \
    PLAN_OPTS="--two-page" \
    bash chain_k.sh "lv$K" "$K" -- --no-plane-drop 2>&1 | grep GRADE
done
