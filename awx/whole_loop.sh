#!/bin/zsh
# whole_loop.sh SOLVE.json OUTDIR [ROUNDS] -- the whole-route plan, every step fed by a MEASUREMENT of the one before:
#   geometry (whole_geo: the joint LP, with every side flip measured so far)
#   -> polish (whole_polish: the audit's own measures met in board xy)
#   -> audit (whole_audit through whole_gate: what the router will be handed)
#   -> side flips the polish could not avoid (an island it found no room beside, or one holding a lane's approach off
#      its stub): the geometry again on the SAME solve, before anything is snapped
#   -> a smooth plan that passes: its PAIRS laid first as the pair router moves (whole_snap --pairs), the singles
#      fitted round them (whole_polish, the pairs held) and SNAPPED onto the router's grid (whole_snap); audited, gated
#      and linted (the pair router's turning radius and straight dives) -- done
#   -> else the island / via cuts the geometry could not meet, and the via cuts of the pair dives the polish could not
#      lay straight: the solve again (warm, every cut so far)
# The bench from BENCH / NETS / DEST (whole_ctx), under the chain's plan environment (set below).
# SEED_FLIPS / SEED_CUTS: comma lists of earlier polish / geometry JSONs to start from.
# A loop that is NOT CONVERGING stops: each round's smooth plan is measured by its audit findings (dive, static,
# shape, swim, pitch in the plan, and any length outside its band), and two rounds in a row that fail to beat the
# best so far end it -- flips and cuts that only move the findings about are not getting there, and every cut round
# pays a solve (one such run went 6 -> 9 -> 10 findings, its cut solve 36 -> 46 vias).
# Exit 0 with OUTDIR/plan.json the snapped plan that passed; 1 when no round got there; 3 when it stopped not
# converging.
HERE=${0:A:h}
cd $HERE
export PLAN_PAGES=1 PLAN_JUDGE=count PLAN_JUDGE_LEN=lane BRAID_PAIRS=1 PLAN_PAIRS=1 BRAID_EXACT_PAGES=0 PLAN_PAGES_SIDERS=2
solve=${1:A}; out=${2:A}; rounds=${3:-6}
mkdir -p $out
flips="${SEED_FLIPS:-}"; cuts="${SEED_CUTS:-}"
nflips() { python3 -c "import json,sys; print(len(json.load(open(sys.argv[1])).get('flips', [])))" $1; }
# the findings in a gate line (whole_gate's summary): dive, static, shape, swim, pitch in the plan, band outside (0/1)
findings() { python3 -c "
import re, sys
s = sys.argv[1]
n = sum(int(re.search(k + r' (\d+)', s).group(1)) for k in ('dive', 'static', 'shape', 'swim'))
n += int(re.search(r'pitch (\d+) in the plan', s).group(1))
print(n + (1 if float(re.search(r'band ([\d.]+) mm', s).group(1)) > 0 else 0))" "$1"; }
PATIENCE=2                                 # rounds in a row without a new best
best=-1; best_i=0; stall=0
for i in $(seq 1 $rounds); do
  echo "=== round $i: geometry of $(basename $solve)${flips:+ (flips from $(basename ${flips##*,}))}"
  GEO_FLIPS_FROM=$flips python3 whole_geo.py $solve $out/g$i.json > $out/g$i.log 2>&1 || { tail -3 $out/g$i.log; exit 1; }
  python3 whole_polish.py $out/g$i.json $out/p$i.json > $out/p$i.log 2>&1 || { tail -3 $out/p$i.log; exit 1; }
  python3 whole_audit.py $out/p$i.json > $out/p$i.audit 2>&1 || { tail -3 $out/p$i.audit; exit 1; }
  gl=$(python3 whole_gate.py $out/p$i.json $out/p$i.audit)
  echo "$gl" | sed 's/^/  smooth: /'
  f=$(findings "$gl")
  if [ $best -lt 0 ] || [ $f -lt $best ]; then best=$f; best_i=$i; stall=0; else stall=$((stall + 1)); fi
  if [ $stall -ge $PATIENCE ]; then
    echo "=== round $i: NOT CONVERGING -- $f finding(s), the best $best at round $best_i, $PATIENCE rounds without a better one"
    exit 3
  fi
  before=$([ -n "$flips" ] && nflips ${flips##*,} || echo 0)
  after=$(nflips $out/p$i.json)
  if [ "$after" -gt "$before" ]; then
    flips=$out/p$i.json                    # the polish output carries every flip so far
    echo "=== round $i: $((after - before)) new side flip(s) -> the geometry again on the same solve"
    continue
  fi
  if python3 whole_gate.py $out/p$i.json $out/p$i.audit > /dev/null; then
    # the PAIRS first, laid as the pair router moves (its turning radius, its straight dives), then the singles
    # fitted round them (the polish, the pairs held) and snapped
    echo "=== round $i: the smooth plan passes -> the pairs laid first"
    python3 whole_snap.py $out/p$i.json $out/pairs$i.json --pairs > $out/pairs$i.log 2>&1
    grep -E "^snap:|FAILED" $out/pairs$i.log | sed 's/^/  /'
    grep -q "^SNAP FAILED" $out/pairs$i.log && { echo "=== round $i: a pair cannot be laid"; exit 1; }
    python3 whole_polish.py $out/pairs$i.json $out/q$i.json > $out/q$i.log 2>&1 || { tail -3 $out/q$i.log; exit 1; }
    python3 whole_audit.py $out/q$i.json > $out/q$i.audit 2>&1 || { tail -3 $out/q$i.audit; exit 1; }
    python3 whole_gate.py $out/q$i.json $out/q$i.audit | sed 's/^/  pairs held: /'
    python3 whole_gate.py $out/q$i.json $out/q$i.audit > /dev/null || { echo "=== round $i: the singles do not fit round the pairs"; exit 1; }
    python3 whole_snap.py $out/q$i.json $out/plan.json > $out/snap.log 2>&1
    grep -E "^snap:|FAILED" $out/snap.log | sed 's/^/  /'
    python3 whole_audit.py $out/plan.json > $out/plan.audit 2>&1 || { tail -3 $out/plan.audit; exit 1; }
    python3 whole_gate.py $out/plan.json $out/plan.audit | sed 's/^/  snapped: /'
    lint=$(python3 whole_lint.py $out/plan.json | tail -1)
    echo "  snapped: $lint"
    python3 whole_gate.py $out/plan.json $out/plan.audit > /dev/null && [ "$lint" = "LINT clean" ] && { echo "=== the plan passes: $out/plan.json"; exit 0; }
    echo "=== round $i: the snapped plan does not pass"; exit 1
  fi
  n=$(python3 -c "import json; d=json.load(open('$out/g$i.json')); p=json.load(open('$out/p$i.json')); print(len(d.get('cuts', [])) + len(d.get('vcuts', [])) + len(p.get('vcuts', [])))")
  if [ "$n" = "0" ]; then echo "=== round $i: no flips and no cuts left"; exit 1; fi
  cuts="${cuts:+$cuts,}$out/g$i.json,$out/p$i.json"
  echo "=== round $i: $n cut(s) -> the solve again"
  HINT=$solve CUTS=$cuts python3 whole_solve.py $out/s$((i + 1)).json 2>&1 | grep -E "whole_solve|vias|check" | sed 's/^/  /'
  solve=$out/s$((i + 1)).json
done
echo "=== no round passed"
exit 1
