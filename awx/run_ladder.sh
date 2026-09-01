#!/bin/bash
# #622 solver ladder: per K -- solve, oracle fanout, chain both arms.
# All outputs land under tmp/ (gitignored; chain_k.sh prefixes its
# own TAG outputs the same way).
cd "$(dirname "$0")"
mkdir -p tmp
for K in "$@"; do
  echo "===== K$K ====="
  python3 plan_global.py solve --k "$K" --out-prefix "tmp/solved_k$K" \
    2>&1 | grep -E 'SOLVED|PENDING|PINNED'
  python3 plan_fanout.py "$K" \
    --contract-json "tmp/solved_k${K}_contract.json" \
    --out "tmp/plan_sv$K.kicad_pcb" > "tmp/plan_sv$K.log" 2>&1
  grep -E 'PLAN OBEDIENCE.*rescue|DRC after' "tmp/plan_sv$K.log" \
    | sed 's/^/  /'
  TWO_PAGE=1 TP_SCOPE=split PLAN_OPTS="--two-page" \
    bash chain_k.sh "lc$K" "$K" -- --no-plane-drop 2>&1 | grep GRADE
  BASE="tmp/plan_sv$K.kicad_pcb" TWO_PAGE=1 TP_SCOPE=split \
    PLAN_OPTS="--two-page" \
    bash chain_k.sh "lv$K" "$K" -- --no-plane-drop 2>&1 | grep GRADE
done
