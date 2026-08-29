#!/bin/bash
# Baseline the BRAID ALONE (no A*, no hybrid) on the COHERENT K-ladder
# checkpoints, so the scaling ceiling is measured on the problem the
# campaign actually defined -- a prefix that never splits a river.
cd "$(dirname "$0")"
TAG=${1:-t4}
shift
for K in "$@"; do
  NETS=$(python3 coherent_nets.py "$K")
  echo "=== K$K  $(date +%H:%M:%S)"
  timeout_guard=0
  python3 -u topo_emit.py --nets "$NETS" --out "${TAG}_k${K}" \
    > "${TAG}_k${K}.log" 2>&1
  if [ -f "${TAG}_k${K}.kicad_pcb" ]; then
    python3 grade_k.py "${TAG}_k${K}.kicad_pcb" "$NETS"
  else
    echo "  NO BOARD: $(tail -3 "${TAG}_k${K}.log" | head -2)"
  fi
done
echo "=== braid ladder done $(date +%H:%M:%S)"
