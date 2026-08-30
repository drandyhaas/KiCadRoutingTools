#!/bin/bash
# The full chain with this session's planning work in it:
#
#   1. plan both ends (plan_ends), every face of the destination on
#      offer (the corridor braid delivers to any of them). Side hints
#      only: the exit LINE hint was measured to resolve two nets into
#      one physical gap (K15: SDQM1/SDQ14 stubs ending at ONE point),
#      so the plan owns the corridor and the braid owns the lane order.
#      DIRS=left,down restricts the plan to the faces the pre-corridor
#      braid could deliver to (the A/B arm);
#   2. fan out the DESTINATION array in the planned directions
#      (production bga_fanout, via escape_dir_hints), and grade THAT
#      board alone -- a berth that ships stub-vs-stub contact is
#      broken before the braid starts;
#   3. braid (braid.py) from the source teeth to the destination's STUB
#      ENDS: corridors from the geometry, a spine per corridor, order
#      and layers from the schedule, and every lane routed by the real
#      router (connect.py) inside its band. No hand tails, no
#      fallbacks: a refused lane is reported, not patched.
#
# Graded exactly like the plain braid ladder: check_connected scoped to
# the run's nets, whole-board check_drc at the routed floor. Extra
# arguments after the Ks go to fanout_from_plan (e.g. --no-plane-drop,
# which isolates the braid's own DRC from the plane-drop pass's known
# cap collisions).
#
# usage: [DIRS=left,down] chain_k.sh TAG K [K...] [-- fanout options]
cd "$(dirname "$0")"
TAG=${1:-chain}
shift
# BASE: the bare bench to chain on (default fb_t2q_base). The rotation
# proof sets it to a rotated copy -- the whole chain must then come out
# with the same grade, or something is leaning on the board's axes.
BASE=${BASE:-fb_t2q_base.kicad_pcb}
DEST=${DEST:-DU1}
DIRS_OPT=${DIRS:+--dirs=$DIRS}
KS=()
while [ $# -gt 0 ] && [ "$1" != "--" ]; do KS+=("$1"); shift; done
[ "$1" = "--" ] && shift
FO_OPTS=("$@")
for K in "${KS[@]}"; do
  echo "=== K$K  $(date +%H:%M:%S)"
  NETS=$(python3 coherent_nets.py "$K")
  python3 fanout_from_plan.py "${TAG}_fo_k${K}.kicad_pcb" "$K" \
    --board="$BASE" $DIRS_OPT --no-lines "${FO_OPTS[@]}" \
    > "${TAG}_fo_k${K}.log" 2>&1
  grep -E "kept floor|^plan:|obeyed|failed nets" "${TAG}_fo_k${K}.log" \
    | sed 's/^/  /'
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
