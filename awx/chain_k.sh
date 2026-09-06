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
TAG=${1:-chain}
shift
mkdir -p tmp
case "$TAG" in */*) ;; *) TAG="tmp/$TAG";; esac
BASE=${BASE:-fb_t2q_fresh.kicad_pcb}
DEST=${DEST:-DU1}
for K in "$@"; do
  echo "=== K$K  $(date +%H:%M:%S)"
  # fresh outputs per run: a reused tag would re-read its own previous
  # .kicad_pro DRC floor, and a crashed stage would grade LAST run's
  # board as this run's. The chain itself is bit-deterministic.
  rm -f "${TAG}_fo_k${K}.kicad_pcb" "${TAG}_fo_k${K}.kicad_pro" \
        "${TAG}_k${K}.kicad_pcb" "${TAG}_k${K}.kicad_pro"
  NETS=$(python3 coherent_nets.py "$K")
  python3 fanout_from_plan.py "${TAG}_fo_k${K}.kicad_pcb" "$K" \
    --board="$BASE" > "${TAG}_fo_k${K}.log" 2>&1
  grep -E "^plan|^wrote|^  round|^  kept|^  destination|source realize:|audit:|ORDER|plan model total" "${TAG}_fo_k${K}.log" | sed 's/^/  /'
  if [ ! -f "${TAG}_fo_k${K}.kicad_pcb" ]; then
    echo "  NO FANOUT BOARD"; continue
  fi
  echo -n "  fanout board: "
  python3 ../py_router/check_drc.py "${TAG}_fo_k${K}.kicad_pcb" \
    --clearance 0.1 --clearance-margin 0.1 2>&1 | grep -E "FOUND|NO DRC"
  python3 -u braid.py --board "${TAG}_fo_k${K}.kicad_pcb" \
    --dest "$DEST" --nets "$NETS" --out "${TAG}_k${K}" \
    > "${TAG}_k${K}.log" 2>&1
  if [ -f "${TAG}_k${K}.kicad_pcb" ]; then
    grep -E "WARNING|violations$" "${TAG}_k${K}.log" | sed 's/^/  /'
    python3 grade_k.py "${TAG}_k${K}.kicad_pcb" "$NETS"
  else
    echo "  NO BRAID: $(grep -E 'Error|assert|Traceback' -m1 -A1 \
      "${TAG}_k${K}.log" | tail -1)"
  fi
done
echo "=== chain done $(date +%H:%M:%S)"
