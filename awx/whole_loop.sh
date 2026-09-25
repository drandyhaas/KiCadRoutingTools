#!/bin/zsh
# whole_loop.sh SOLVE.json OUTDIR [ROUNDS] -- the whole-route plan, every step fed by a MEASUREMENT of the one before:
#   geometry (whole_geo: the joint LP, with every side flip measured so far)
#   -> polish (whole_polish: the audit's own measures met in board xy)
#   -> audit (whole_audit through whole_gate: what the router will be handed)
#   -> side flips the polish could not avoid (an island it found no room beside, or one holding a lane's approach off
#      its stub): the geometry again on the SAME solve, before anything is snapped
#   -> a smooth plan that passes: SNAP it onto the router's grid (whole_snap), audit and gate it again -- done
#   -> else the island / via cuts the geometry could not meet: the solve again (warm, every cut so far)
# The bench from BENCH / NETS / DEST (whole_ctx), under the chain's plan environment (set below).
# SEED_FLIPS / SEED_CUTS: comma lists of earlier polish / geometry JSONs to start from.
# Exit 0 with OUTDIR/plan.json the snapped plan that passed; 1 when no round got there.
HERE=${0:A:h}
cd $HERE
export PLAN_PAGES=1 PLAN_JUDGE=count PLAN_JUDGE_LEN=lane BRAID_PAIRS=1 PLAN_PAIRS=1 BRAID_EXACT_PAGES=0 PLAN_PAGES_SIDERS=2
solve=${1:A}; out=${2:A}; rounds=${3:-6}
mkdir -p $out
flips="${SEED_FLIPS:-}"; cuts="${SEED_CUTS:-}"
nflips() { python3 -c "import json,sys; print(len(json.load(open(sys.argv[1])).get('flips', [])))" $1; }
for i in $(seq 1 $rounds); do
  echo "=== round $i: geometry of $(basename $solve)${flips:+ (flips from $(basename ${flips##*,}))}"
  GEO_FLIPS_FROM=$flips python3 whole_geo.py $solve $out/g$i.json > $out/g$i.log 2>&1 || { tail -3 $out/g$i.log; exit 1; }
  python3 whole_polish.py $out/g$i.json $out/p$i.json > $out/p$i.log 2>&1 || { tail -3 $out/p$i.log; exit 1; }
  python3 whole_audit.py $out/p$i.json > $out/p$i.audit 2>&1 || { tail -3 $out/p$i.audit; exit 1; }
  python3 whole_gate.py $out/p$i.json $out/p$i.audit | sed 's/^/  smooth: /'
  before=$([ -n "$flips" ] && nflips ${flips##*,} || echo 0)
  after=$(nflips $out/p$i.json)
  if [ "$after" -gt "$before" ]; then
    flips=$out/p$i.json                    # the polish output carries every flip so far
    echo "=== round $i: $((after - before)) new side flip(s) -> the geometry again on the same solve"
    continue
  fi
  if python3 whole_gate.py $out/p$i.json $out/p$i.audit > /dev/null; then
    echo "=== round $i: the smooth plan passes -> snap"
    python3 whole_snap.py $out/p$i.json $out/plan.json > $out/snap.log 2>&1
    grep -E "^snap:|FAILED" $out/snap.log | sed 's/^/  /'
    python3 whole_audit.py $out/plan.json > $out/plan.audit 2>&1 || { tail -3 $out/plan.audit; exit 1; }
    python3 whole_gate.py $out/plan.json $out/plan.audit | sed 's/^/  snapped: /'
    python3 whole_lint.py $out/plan.json | tail -1 | sed 's/^/  snapped: /'
    python3 whole_gate.py $out/plan.json $out/plan.audit > /dev/null && { echo "=== the plan passes: $out/plan.json"; exit 0; }
    echo "=== round $i: the snapped plan does not pass"; exit 1
  fi
  n=$(python3 -c "import json; d=json.load(open('$out/g$i.json')); print(len(d.get('cuts', [])) + len(d.get('vcuts', [])))")
  if [ "$n" = "0" ]; then echo "=== round $i: no flips and no cuts left"; exit 1; fi
  cuts="${cuts:+$cuts,}$out/g$i.json"
  echo "=== round $i: $n cut(s) -> the solve again"
  HINT=$solve CUTS=$cuts python3 whole_solve.py $out/s$((i + 1)).json 2>&1 | grep -E "whole_solve|vias|check" | sed 's/^/  /'
  solve=$out/s$((i + 1)).json
done
echo "=== no round passed"
exit 1
