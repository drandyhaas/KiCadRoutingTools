#!/bin/bash
# Emit one coherent-ladder checkpoint with the braid alone.
# Usage: run_k.sh K OUT [extra topo_emit args...]
cd "$(dirname "$0")"
K=$1; OUT=$2; shift 2
python3 -u topo_emit.py --nets "$(python3 coherent_nets.py "$K")" \
  --out "$OUT" "$@"
