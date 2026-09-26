#!/bin/zsh
# whole_loop.sh SOLVE.json OUTDIR [ROUNDS] -- the whole-route plan, every step fed by a MEASUREMENT of the one before:
#   geometry (whole_geo: the joint LP, with every side flip measured so far)
#   -> polish (whole_polish: the audit's own measures met in board xy)
#   -> audit (whole_audit through whole_gate: what the router will be handed)
#   -> side flips the polish could not avoid (an island it found no room beside, or one holding a lane's approach off
#      its stub): the geometry again on the SAME solve, before anything is snapped -- or, when the round has cuts or
#      findings as well, the solve again with them and the geometry with the flips, in one round
#   -> a smooth plan that passes: its PAIRS laid first as the pair router moves (whole_snap --pairs), the singles
#      fitted round them (whole_polish, the pairs held) and SNAPPED onto the router's grid (whole_snap); audited, gated
#      and linted (the pair router's turning radius and straight dives) -- done
#   -> else the solve again (warm), with every cut so far -- the island / via cuts the geometry could not meet and the
#      via cuts of the pair dives the polish could not lay straight -- and every audit's findings so far as HISTORY
#      (whole_gate --hot: where the plan was short; the solve prices crossings and changes there by how many audits
#      found it so, whole_solve HIST). A finding with no cut to send -- a pitch, a shape, a dive with the pairs held or
#      snapped -- is still sent: its place, priced.
# The bench from BENCH / NETS / DEST (whole_ctx), under the chain's plan environment (set below).
# SEED_FLIPS / SEED_CUTS / SEED_HIST: comma lists of earlier polish / geometry / hot JSONs to start from. A HARNESS
# that runs the same bench again and again: it turns on the caches the router leaves off (STAGE_CACHE=1: a stage
# already run on the same inputs with the same code restored from awx/tmp/stage_cache, stage_cache.py; TAUT_MEMO=1:
# the braid's taut strings kept under awx/tmp/taut_memo) -- STAGE_CACHE=0 / TAUT_MEMO=0 run it without them.
# A loop that is NOT CONVERGING stops: each round is scored by how far its plan got (smooth, the pairs held, snapped)
# and its audit findings there (dive, static, shape, swim, pitch in the plan, any length outside its band), and two
# rounds in a row that fail to beat the best so far end it -- flips, cuts and prices that only move the findings about
# are not getting there, and every such round pays a solve.
# Exit 0 with OUTDIR/plan.json the snapped plan that passed; 1 when no round got there; 3 when it stopped not
# converging.
HERE=${0:A:h}
solve=${1:A}; out=${2:A}; rounds=${3:-6}
cd $HERE
# the braid's plan environment the planning reads (a pages-first sidecar's paging, its pairs)
export BRAID_PAIRS=1 BRAID_EXACT_PAGES=0 PLAN_PAGES_SIDERS=2
# every expensive stage through stage_cache.py: a stage whose script, arguments, environment and every file it read
# are unchanged is restored, not run -- on here, a harness's cache (STAGE_CACHE=0 runs them all)
export STAGE_CACHE=${STAGE_CACHE:-1} TAUT_MEMO=${TAUT_MEMO:-1}
ST=(python3 stage_cache.py)
mkdir -p $out
flips="${SEED_FLIPS:-}"; cuts="${SEED_CUTS:-}"; hist="${SEED_HIST:-}"
# the side flips in a comma list of polish outputs, every file's (a SEED_FLIPS list names several)
nflips() { python3 -c "import json,sys; print(len({tuple(x) for f in sys.argv[1].split(',') if f for x in json.load(open(f)).get('flips', [])}))" "$1"; }
# the findings in a gate line (whole_gate's summary): dive, static, shape, swim, pitch in the plan, band outside (0/1);
# a gate line without its counts (an audit that did not run to its end) counts as many
findings() { python3 -c "
import re, sys
s = sys.argv[1]
try:
    n = sum(int(re.search(k + r' (\d+)', s).group(1)) for k in ('dive', 'static', 'shape', 'swim'))
    n += int(re.search(r'pitch (\d+) in the plan', s).group(1))
    n += int(re.search(r'band broken (\d+)', s).group(1)) if 'band broken' in s else 0
    print(n + (1 if float(re.search(r'band ([\d.]+) mm', s).group(1)) > 0 else 0))
except AttributeError:
    print(999)" "$1"; }
# an audit's findings as history: its hot places (whole_gate --hot), added when it names any
addhot() {
  python3 whole_gate.py $1 $2 --hot $3 > /dev/null
  [ "$(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))['hot']))" $3)" = "0" ] && return 1
  hist="${hist:+$hist,}$3"
}
# the solve again, warm, with every cut and every audit's history so far -- less an island cut a side flip has
# answered since (the flip puts that lane on the island's other side; the cut would keep holding it off the island)
resolve() {
  local s2=$out/s$((i + 1)).json
  python3 - "$cuts" "$flips" "$out/cuts$((i + 1)).json" <<'PY'
import json, sys
flipped = {tuple(x) for f in sys.argv[2].split(',') if f for x in json.load(open(f)).get('flips', [])}
cuts, vcuts = [], []
for f in [f for f in sys.argv[1].split(',') if f]:
    c = json.load(open(f))
    cuts += [x for x in c.get('cuts', []) if (x['lane'], x['island']) not in flipped]
    vcuts += c.get('vcuts', [])
json.dump({'cuts': cuts, 'vcuts': vcuts}, open(sys.argv[3], 'w'))
PY
  HINT=$solve CUTS=$out/cuts$((i + 1)).json HIST=$hist $ST --out $s2 -- whole_solve.py $s2 > ${s2%.json}.log 2>&1 || { tail -3 ${s2%.json}.log; exit 1; }
  grep -E "whole_solve|vias|check|history" ${s2%.json}.log | sed 's/^/  /'
  solve=$s2
}
# the round's score -- how far its plan got (a stage not reached a thousand) and its findings there -- against the best
PATIENCE=2                                 # rounds in a row without a new best
best=-1; best_i=0; stall=0
progress() {
  if [ $best -lt 0 ] || [ $1 -lt $best ]; then best=$1; best_i=$i; stall=0; else stall=$((stall + 1)); fi
  if [ $stall -ge $PATIENCE ]; then
    echo "=== round $i: NOT CONVERGING -- score $1, the best $best at round $best_i, $PATIENCE rounds without a better one"
    exit 3
  fi
}
for i in $(seq 1 $rounds); do
  echo "=== round $i: geometry of $(basename $solve)${flips:+ (flips from $(basename ${flips##*,}))}"
  GEO_FLIPS_FROM=$flips $ST --out $out/g$i.json -- whole_geo.py $solve $out/g$i.json > $out/g$i.log 2>&1 || { tail -3 $out/g$i.log; exit 1; }
  $ST --out $out/p$i.json -- whole_polish.py $out/g$i.json $out/p$i.json > $out/p$i.log 2>&1 || { tail -3 $out/p$i.log; exit 1; }
  $ST -- whole_audit.py $out/p$i.json > $out/p$i.audit 2>&1 || { tail -3 $out/p$i.audit; exit 1; }
  gl=$(python3 whole_gate.py $out/p$i.json $out/p$i.audit)
  echo "$gl" | sed 's/^/  smooth: /'
  f=$(findings "$gl")
  before=$([ -n "$flips" ] && nflips "$flips" || echo 0)
  after=$(nflips $out/p$i.json)
  # the round's cuts -- the geometry's islands and via cuts, the polish's via cuts -- less an island cut that one of
  # the round's NEW flips answers (the flip puts that lane on the island's other side; the geometry has not tried it)
  n=$(python3 - "$out/g$i.json" "$out/p$i.json" "$flips" "$out/c$i.json" <<'PY'
import json, sys
g, p = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
old = {tuple(x) for f in sys.argv[3].split(',') if f for x in json.load(open(f)).get('flips', [])}
new = {tuple(x) for x in p.get('flips', [])} - old
cuts = [c for c in g.get('cuts', []) if (c['lane'], c['island']) not in new]
vcuts = g.get('vcuts', []) + p.get('vcuts', [])
json.dump({'cuts': cuts, 'vcuts': vcuts}, open(sys.argv[4], 'w'))
print(len(cuts) + len(vcuts))
PY
)
  passes=$(python3 whole_gate.py $out/p$i.json $out/p$i.audit > /dev/null && echo 1 || echo 0)
  [ $passes = 0 ] && addhot $out/p$i.json $out/p$i.audit $out/hp$i.json && n=$((n + 1))
  if [ "$after" -gt "$before" ]; then
    progress $((2000 + f))
    flips=$out/p$i.json                    # the polish output carries every flip so far
    if [ "$n" = "0" ]; then
      echo "=== round $i: $((after - before)) new side flip(s) -> the geometry again on the same solve"
      continue
    fi
    # flips AND cuts or findings: both at once -- the solve with them, then the geometry with the flips (one round)
    cuts="${cuts:+$cuts,}$out/c$i.json"
    echo "=== round $i: $((after - before)) new side flip(s), and cuts or findings -> the solve again, then the geometry with the flips"
    resolve
    continue
  fi
  if [ $passes = 1 ]; then
    # the PAIRS first, laid as the pair router moves (its turning radius, its straight dives), then the singles
    # fitted round them (the polish, the pairs held) and snapped
    echo "=== round $i: the smooth plan passes -> the pairs laid first"
    $ST --out $out/pairs$i.json -- whole_snap.py $out/p$i.json $out/pairs$i.json --pairs > $out/pairs$i.log 2>&1 || { tail -3 $out/pairs$i.log; exit 1; }
    grep -E "^snap:|FAILED" $out/pairs$i.log | sed 's/^/  /'
    grep -q "^SNAP FAILED" $out/pairs$i.log && { echo "=== round $i: a pair cannot be laid"; exit 1; }
    $ST --out $out/q$i.json -- whole_polish.py $out/pairs$i.json $out/q$i.json > $out/q$i.log 2>&1 || { tail -3 $out/q$i.log; exit 1; }
    $ST -- whole_audit.py $out/q$i.json > $out/q$i.audit 2>&1 || { tail -3 $out/q$i.audit; exit 1; }
    gq=$(python3 whole_gate.py $out/q$i.json $out/q$i.audit)
    echo "$gq" | sed 's/^/  pairs held: /'
    if ! python3 whole_gate.py $out/q$i.json $out/q$i.audit > /dev/null; then
      # the singles do not fit round the pairs: where they are short goes to the solve as history
      progress $((1000 + $(findings "$gq")))
      addhot $out/q$i.json $out/q$i.audit $out/hq$i.json || { echo "=== round $i: the singles do not fit round the pairs"; exit 1; }
      echo "=== round $i: the singles do not fit round the pairs -> the solve again, their places priced"
      resolve
      continue
    fi
    $ST --out $out/plan.json -- whole_snap.py $out/q$i.json $out/plan.json > $out/snap.log 2>&1 || { tail -3 $out/snap.log; exit 1; }
    grep -E "^snap:|FAILED" $out/snap.log | sed 's/^/  /'
    $ST -- whole_audit.py $out/plan.json > $out/plan.audit 2>&1 || { tail -3 $out/plan.audit; exit 1; }
    gs=$(python3 whole_gate.py $out/plan.json $out/plan.audit)
    echo "$gs" | sed 's/^/  snapped: /'
    lint=$(python3 whole_lint.py $out/plan.json | tail -1)
    echo "  snapped: $lint"
    python3 whole_gate.py $out/plan.json $out/plan.audit > /dev/null && [ "$lint" = "LINT clean" ] && { echo "=== the plan passes: $out/plan.json"; exit 0; }
    # the snapped plan is short: where goes to the solve as history (a lint finding has no place: that stops)
    progress $(findings "$gs")
    addhot $out/plan.json $out/plan.audit $out/hs$i.json || { echo "=== round $i: the snapped plan does not pass"; exit 1; }
    echo "=== round $i: the snapped plan does not pass -> the solve again, its places priced"
    resolve
    continue
  fi
  progress $((2000 + f))
  if [ "$n" = "0" ]; then echo "=== round $i: no flips, no cuts and no findings with a place left"; exit 1; fi
  cuts="${cuts:+$cuts,}$out/c$i.json"
  echo "=== round $i: cuts or findings -> the solve again"
  resolve
done
echo "=== no round passed"
exit 1
