#!/bin/bash
# chain_k.sh TAG K [K...]  --  the K28 chain: plan + fan out the
# destination in the planned directions, braid every lane, grade.
#
#   1. fanout_from_plan.py: plan both ends (plan_ends), fan out the
#      DESTINATION array with the production engine in the planned
#      directions, and grade THAT board alone -- a berth that ships
#      stub-vs-stub contact is broken before the braid starts;
#   2. braid.py: from the source teeth to the destination's stub ends,
#      corridors from the geometry, a spine per corridor, order and
#      layers from the schedule, every lane routed by the real router
#      (connect.py) inside its band. A refused lane is reported, not
#      patched.
#
# Graded with check_connected scoped to the run's nets and whole-board
# check_drc at the routed floor. Outputs (boards, logs, sidecars) live
# under tmp/ (gitignored) unless TAG contains a slash.
#
# BASE (default fb_t2q_fresh.kicad_pcb): the bench, its source array
# already fanned out. DEST (default DU1): the destination reference.
cd "$(dirname "$0")"
# ONE BLAS THREAD. Apple's Accelerate (and OpenBLAS/MKL elsewhere) sizes
# its own pool from the machine and the load, which makes a reduction's
# summation order machine-dependent -- the same class of defect as a
# wall-clock budget, and this chain is required to be deterministic.
# HiGHS already pins itself to one thread; CP-SAT's 4 workers are
# reproducible because they run under max_deterministic_time. Pinning
# also stops the residue search's 6 worker PROCESSES from each spawning
# a BLAS pool on an 8-core box.
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-1}
export VECLIB_MAXIMUM_THREADS=${VECLIB_MAXIMUM_THREADS:-1}
export OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS:-1}
export MKL_NUM_THREADS=${MKL_NUM_THREADS:-1}
export NUMEXPR_NUM_THREADS=${NUMEXPR_NUM_THREADS:-1}
TAG=${1:-chain}
shift
mkdir -p tmp
case "$TAG" in */*) ;; *) TAG="tmp/$TAG";; esac
BASE=${BASE:-fb_t2q_fresh.kicad_pcb}
DEST=${DEST:-DU1}
# THE DESIGN CONSTANTS, from the one place they are defined (rules.py) --
# this used to be a literal 0.1 repeated here and in grade_k.py. Printed,
# because a grade whose clearance is invisible is a number nobody can check.
RULES_OUT=$(python3 rules.py)
CLR=$(printf '%s\n' "$RULES_OUT" | awk '$1=="clearance"{print $2; exit}')
CLR=${CLR:-0.1}
printf '=== %s\n' "$RULES_OUT"
echo "=== grading at clearance $CLR"
for K in "$@"; do
  echo "=== K$K  $(date +%H:%M:%S)"
  # fresh outputs per run: a reused tag would re-read its own previous
  # .kicad_pro DRC floor, and a crashed stage would grade LAST run's
  # board as this run's. The chain itself is bit-deterministic.
  rm -f "${TAG}_fo_k${K}.kicad_pcb" "${TAG}_fo_k${K}.kicad_pro" \
        "${TAG}_k${K}.kicad_pcb" "${TAG}_k${K}.kicad_pro"
  NETS=$(python3 coherent_nets.py "$K" --board="$BASE")
  # THE FLOW FRAME (flow_frame.py): the pair turned, as a file, by the
  # exact quarter turn that points source-to-destination along +x; every
  # stage runs on that file and the result is turned back, so a pair
  # dropped at any of the four angles is the identical computation. k=0
  # (the bench) runs on the base itself, unchanged.
  read FK FCX FCY <<< "$(python3 flow_frame.py quarter "$BASE" "$DEST" "$NETS" 2>/dev/null | tail -1)"
  RUNBASE="$BASE"
  if [ -n "$FK" ] && [ "$FK" != "0" ]; then
    RUNBASE="${TAG}_frame_k${K}.kicad_pcb"
    rm -f "$RUNBASE"
    python3 flow_frame.py turn "$BASE" "$RUNBASE" "$FK" "$FCX" "$FCY" > /dev/null 2>&1
    if [ ! -f "$RUNBASE" ]; then echo "  FLOW FRAME: turn failed"; continue; fi
    echo "  flow frame: $FK quarter turn(s) about ($FCX, $FCY) -> $(basename "$RUNBASE")"
  fi
  # CHAIN_FANOUT_AB=1 (2026-09-15, session 13): plan and fan out BOTH ways
  # -- with and without the JOINT SOURCE RE-FAN (SRC_REFAN_JOINT) -- and
  # carry every DISTINCT board into the braid portfolio below. The joint
  # re-fan frees the nets whose copper stands in a source move's room so the
  # engine can rip and re-lay them around the ask, and measured on the four
  # rungs it changes the board on only TWO of them: at K51 it reaches the
  # board that routes 98 (against 115), at K35 a board that routes one via
  # worse, and at K28/K41 it is inert -- the boards are copper-IDENTICAL, so
  # the identity check below costs the extra braids nothing there.
  FOS=""
  if [ "${CHAIN_FANOUT_AB:-0}" != "0" ]; then
    for J in 0 1; do
      rm -f "${TAG}_fo_k${K}_J${J}.kicad_pcb" "${TAG}_fo_k${K}_J${J}.kicad_pro" \
            "${TAG}_fo_k${K}_J${J}.plan.json"
      SRC_REFAN_JOINT=$J python3 fanout_from_plan.py "${TAG}_fo_k${K}_J${J}.kicad_pcb" "$K" \
        --board="$RUNBASE" > "${TAG}_fo_k${K}_J${J}.log" 2>&1
      [ -f "${TAG}_fo_k${K}_J${J}.kicad_pcb" ] && FOS="$FOS ${TAG}_fo_k${K}_J${J}"
    done
    # a board identical to one already in the list is not a candidate
    FOS=$(python3 dedupe_boards.py $FOS)
    echo "  fanout A/B: $(echo $FOS | wc -w | tr -d ' ') distinct board(s):$FOS"
    # a portfolio with no judge is just "keep the first", so the fanout
    # level turns the braid level on rather than silently picking an arm
    CHAIN_BRAID_AB=${CHAIN_BRAID_AB:-1}
  else
    python3 fanout_from_plan.py "${TAG}_fo_k${K}.kicad_pcb" "$K" \
      --board="$RUNBASE" > "${TAG}_fo_k${K}.log" 2>&1
    FOS="${TAG}_fo_k${K}"
  fi
  if [ -z "$FOS" ]; then
    echo "  NO FANOUT BOARD"; continue
  fi
  for FO in $FOS; do
    grep -E "^plan|^wrote|^  round|^  kept|^  destination|source realize:|audit:|ORDER|plan model total" \
      "$FO.log" | sed "s|^|  $(basename "$FO"): |"
  done
  echo "  fanout stage done $(date +%H:%M:%S)"
  for FO in $FOS; do
    echo -n "  fanout board $(basename "$FO"): "
    python3 ../py_router/check_drc.py "$FO.kicad_pcb" \
      --clearance "$CLR" --clearance-margin 0.1 2>&1 | grep -E "FOUND|NO DRC"
  done
  # CHAIN_BRAID_AB=1 (2026-09-15, session 13): braid the fanout board BOTH
  # WAYS and keep the better copper. The two arms differ only in the plan
  # sidecar's `pages_first` marker, which switches on the side-face comb
  # (PLAN_PAGES_SIDERS) and the exact page assignment
  # (schedule.EXACT_PAGES). Measured at K51: the SAME fanout board routes
  # 112 vias with SDQ11 open under the marker and 98 vias with nothing open
  # without it -- and on another board the sidecar is worth 7 vias the other
  # way, so neither arm is "the" answer and a K-dependent default would be a
  # board-specific hack. Routing both and keeping the better is general, it
  # cannot regress, and it costs one braid. The verdict is (open, vias):
  # completion first, as every grade in this chain is.
  if [ "${CHAIN_BRAID_AB:-0}" != "0" ]; then
    # ONE AT A TIME: two braids in parallel is the thing this box cannot do
    # (8 GB), and a concurrent run is also how a deterministic stage stops
    # being one. Each candidate is named after the fanout board it came
    # from plus its arm, so the winner NAMES its own board and regime and
    # nothing has to be parsed back out of an index.
    CANDS=""
    for FO in $FOS; do
      for ARM in A B; do
        case $ARM in
          A) E="" ;;                                      # the sidecar as written
          B) E="BRAID_EXACT_PAGES=0 PLAN_PAGES_SIDERS=0" ;;   # the marker's two rules off
        esac
        rm -f "${FO}_${ARM}".*
        env $E python3 -u braid.py --board "$FO.kicad_pcb" \
          --dest "$DEST" --nets "$NETS" --out "${FO}_${ARM}" \
          > "${FO}_${ARM}.log" 2>&1
        CANDS="$CANDS ${FO}_${ARM}.kicad_pcb"
      done
    done
    win=$(python3 pick_braid.py "$NETS" $CANDS)
    if [ -z "$win" ]; then
      set -- $CANDS
      echo "  braid A/B: NO VERDICT -- keeping $(basename "$1")"
      win="$1"
    fi
    WARM=${win%.kicad_pcb}; WARM=${WARM##*_}        # A | B
    WFO=${win%_${WARM}.kicad_pcb}                   # the fanout board it came from
    cp "$win" "${TAG}_k${K}.kicad_pcb"
    [ -f "${win%.kicad_pcb}.kicad_pro" ] && cp "${win%.kicad_pcb}.kicad_pro" "${TAG}_k${K}.kicad_pro"
    cp "${win%.kicad_pcb}.log" "${TAG}_k${K}.log" 2>/dev/null
    # THE SHIPPED FANOUT BOARD IS THE WINNER'S, AND ITS SIDECAR DESCRIBES
    # THE REGIME THAT ROUTED IT. Two things go wrong otherwise, and both
    # bite the next consumer rather than this run: shipping some OTHER
    # candidate's fanout board leaves `_fo_` and the routed board
    # describing different plans, and shipping a sidecar that still says
    # `pages_first` when the marker-OFF arm won means every later braid of
    # that board -- a re-braid by hand, or `replan.py`, which re-braids F
    # on every round -- routes the arm the portfolio rejected and throws
    # the gain away.
    if [ "$WFO" != "${TAG}_fo_k${K}" ]; then
      cp "$WFO.kicad_pcb" "${TAG}_fo_k${K}.kicad_pcb"
      cp "$WFO.kicad_pro" "${TAG}_fo_k${K}.kicad_pro" 2>/dev/null
      cp "$WFO.log" "${TAG}_fo_k${K}.log" 2>/dev/null
    fi
    if [ "$WARM" = "B" ]; then
      python3 -c "
import json
d = json.load(open('$WFO.plan.json')); d.pop('pages_first', None)
json.dump(d, open('${TAG}_fo_k${K}.plan.json', 'w'))"
      echo "  braid A/B: the marker-OFF arm won -- the shipped sidecar has"\
           "\`pages_first\` removed, so a re-braid (or replan) reproduces it"
    elif [ "$WFO" != "${TAG}_fo_k${K}" ]; then
      cp "$WFO.plan.json" "${TAG}_fo_k${K}.plan.json" 2>/dev/null
    fi
  else
    python3 -u braid.py --board "${TAG}_fo_k${K}.kicad_pcb" \
      --dest "$DEST" --nets "$NETS" --out "${TAG}_k${K}" \
      > "${TAG}_k${K}.log" 2>&1
  fi
  echo "  braid stage done $(date +%H:%M:%S)"
  if [ -f "${TAG}_k${K}.kicad_pcb" ] && [ "$RUNBASE" != "$BASE" ]; then
    # back into the board's own frame (the frame board is kept beside it)
    mv "${TAG}_k${K}.kicad_pcb" "${TAG}_k${K}_frame.kicad_pcb"
    [ -f "${TAG}_k${K}.kicad_pro" ] && mv "${TAG}_k${K}.kicad_pro" "${TAG}_k${K}_frame.kicad_pro"
    python3 flow_frame.py turn "${TAG}_k${K}_frame.kicad_pcb" "${TAG}_k${K}.kicad_pcb" \
      "$((4 - FK))" "$FCX" "$FCY" > /dev/null 2>&1 || echo "  FLOW FRAME: turn back failed"
  fi
  if [ -f "${TAG}_k${K}.kicad_pcb" ]; then
    grep -E "WARNING|violations$" "${TAG}_k${K}.log" | sed 's/^/  /'
    python3 grade_k.py "${TAG}_k${K}.kicad_pcb" "$NETS"
  else
    echo "  NO BRAID: $(grep -E 'Error|assert|Traceback' -m1 -A1 \
      "${TAG}_k${K}.log" | tail -1)"
  fi
done
echo "=== chain done $(date +%H:%M:%S)"
