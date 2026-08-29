#!/bin/bash
# #622 take3: grade an emitted board vs the base (connectivity delta + DRC).
cd "$(dirname "$0")"
for b in fb_t2q_base "${1:-topo_k4_emit}"; do
  echo "== $b"
  python3 ../py_router/check_connected.py "$b.kicad_pcb" 2>&1 \
    | grep -E "\(net [0-9]+\):" | wc -l
  python3 ../py_router/check_connected.py "$b.kicad_pcb" 2>&1 \
    | grep -E "SDQ1[1345] \(net" || echo "  (no K4 nets disconnected)"
  python3 ../py_router/check_drc.py "$b.kicad_pcb" --clearance 0.1 \
    --clearance-margin 0.1 2>&1 | tail -2
done
