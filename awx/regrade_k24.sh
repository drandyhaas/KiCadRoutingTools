#!/bin/bash
# #622: grade the K2/K4 debug arms honestly (connectivity + DRC).
cd "$(dirname "$0")"
for b in zv1_k2 zv2_k2_pb zq1_k4_v2 zq2_k4_v2_pb; do
  echo "== $b"
  python3 ../py_router/check_connected.py $b.kicad_pcb 2>&1 | tail -3
  python3 ../py_router/check_drc.py $b.kicad_pcb --clearance 0.1 --clearance-margin 0.1 2>&1 | tail -2
done
