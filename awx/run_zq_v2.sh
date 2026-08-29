#!/bin/bash
# #622 zl: K=4 back-to-basics eco-line debugging arms.
cd "$(dirname "$0")"
K4="SDQ15 SDQ14 SDQ13 SDQ11"
export KICAD_BUS_EXACT_LEGS=1 KICAD_BUS_DEBUG_ECO=1 KICAD_BUS_BERTHS=1
export KICAD_BUS_FOLLOW_LEADER=1 KICAD_BUS_OFFLANE_MULT=3
export KICAD_ATTRACT_POTENTIAL=10000 KICAD_ATTRACT_BASIN_PCT=50
export KICAD_BUS_MACRO_RIVERS=1 KICAD_BUS_EXIT_EXT=0 KICAD_BUS_SNAP=1
export KICAD_VIA_COLLAPSE=1 KICAD_BUS_DIVE_SPLIT=1 KICAD_BUS_RIBBON=1
export KICAD_BUS_LANE_REPAIR=1 KICAD_BUS_DIVE_BERTHS=0 KICAD_BUS_PLAN_LANES=1 KICAD_BUS_PLAN_VIA_GUARD=1


run_arm () {  # TAG
  local TAG=$1
  local NETS=()
  for n in $K4; do NETS+=("/DDR3 16x1/$n"); done
  python3 -X utf8 ../py_router/route.py fb_t2q_base.kicad_pcb $TAG.kicad_pcb \
    --bus --nets "${NETS[@]}" --ordering bus --bus-attraction-radius 2.5 \
    --clearance 0.1 --track-width 0.1 --via-size 0.25 --via-drill 0.15 \
    --stub-proximity-cost 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 \
    --layer-costs 1.0 1.0 \
    --no-bga-zone > $TAG.log 2>&1
  echo "== $TAG rips=$(grep -c 'Ripping up' $TAG.log)"
}

run_arm zq1_k4_v2
KICAD_BUS_ORDER_LENGTH=planbus run_arm zq2_k4_v2_pb
echo DONE_ZQ > zq.done
