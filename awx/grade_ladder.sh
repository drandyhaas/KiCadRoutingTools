#!/bin/bash
# Net-scoped grade of a K ladder: only the nets each rung actually
# routed (grade_all.sh counts every open net on the board, including
# the DDR nets a smaller rung never touched).
cd "$(dirname "$0")"
TAG=${1:-a}
shift
for K in "$@"; do
  NETS=$(python3 ladder_nets.py "$K")
  python3 grade_k.py "topo_k${K}${TAG}.kicad_pcb" "$NETS"
done
