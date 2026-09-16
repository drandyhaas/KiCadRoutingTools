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
  python3 fanout_from_plan.py "${TAG}_fo_k${K}.kicad_pcb" "$K" \
    --board="$RUNBASE" > "${TAG}_fo_k${K}.log" 2>&1
  grep -E "^plan|^wrote|^  round|^  kept|^  destination|source realize:|audit:|ORDER|plan model total" "${TAG}_fo_k${K}.log" | sed 's/^/  /'
  if [ ! -f "${TAG}_fo_k${K}.kicad_pcb" ]; then
    echo "  NO FANOUT BOARD"; continue
  fi
  echo "  fanout stage done $(date +%H:%M:%S)"
  echo -n "  fanout board: "
  python3 ../py_router/check_drc.py "${TAG}_fo_k${K}.kicad_pcb" \
    --clearance "$CLR" --clearance-margin 0.1 2>&1 | grep -E "FOUND|NO DRC"
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
    # being one.
    rm -f "${TAG}_k${K}_A".* "${TAG}_k${K}_B".*
    python3 -u braid.py --board "${TAG}_fo_k${K}.kicad_pcb" \
      --dest "$DEST" --nets "$NETS" --out "${TAG}_k${K}_A" \
      > "${TAG}_k${K}_A.log" 2>&1
    BRAID_EXACT_PAGES=0 PLAN_PAGES_SIDERS=0 \
    python3 -u braid.py --board "${TAG}_fo_k${K}.kicad_pcb" \
      --dest "$DEST" --nets "$NETS" --out "${TAG}_k${K}_B" \
      > "${TAG}_k${K}_B.log" 2>&1
    win=$(python3 pick_braid.py "$NETS" \
            "${TAG}_k${K}_A.kicad_pcb" "${TAG}_k${K}_B.kicad_pcb")
    case "$win" in
      '') echo "  braid A/B: NO VERDICT -- keeping arm A"; win="${TAG}_k${K}_A.kicad_pcb";;
    esac
    cp "$win" "${TAG}_k${K}.kicad_pcb"
    [ -f "${win%.kicad_pcb}.kicad_pro" ] && cp "${win%.kicad_pcb}.kicad_pro" "${TAG}_k${K}.kicad_pro"
    cp "${win%.kicad_pcb}.log" "${TAG}_k${K}.log" 2>/dev/null
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
