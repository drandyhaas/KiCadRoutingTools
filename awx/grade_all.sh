#!/bin/bash
# #622 take3: grade emitted K-boards (disconnected-net count + DRC tail).
cd "$(dirname "$0")"
for b in "$@"; do
  echo "== $b"
  echo -n "disconnected nets: "
  python3 ../py_router/check_connected.py "$b.kicad_pcb" 2>&1 \
    | grep -cE "\(net [0-9]+\):"
  python3 ../py_router/check_drc.py "$b.kicad_pcb" --clearance 0.1 \
    --clearance-margin 0.1 2>&1 | tail -2 | head -1
done
