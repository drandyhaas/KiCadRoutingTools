#!/bin/bash
# Emit + grade + per-net audit every coherent checkpoint, and print the
# human's via count beside ours. This is the take-4 regression run: it
# must stay at 0 open / 0 drc / 0 structural issues, and the via column
# is the thing to improve.
cd "$(dirname "$0")"
TAG=${1:-w}
shift
for K in "$@"; do
  NETS=$(python3 coherent_nets.py "$K")
  python3 -u topo_emit.py --nets "$NETS" --out "${TAG}_k${K}" \
    > "${TAG}_k${K}.log" 2>&1
  if [ ! -f "${TAG}_k${K}.kicad_pcb" ]; then
    echo "K$K  NO BOARD: $(grep -E 'Error|assert' "${TAG}_k${K}.log" | tail -1)"
    continue
  fi
  python3 grade_k.py "${TAG}_k${K}.kicad_pcb" "$NETS"
  python3 audit_nets.py "${TAG}_k${K}.kicad_pcb" "$K" | tail -1
  python3 human_at_k.py "$K"
done
echo "=== verify done"
