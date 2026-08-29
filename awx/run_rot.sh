#!/bin/bash
# Emit a coherent checkpoint on a ROTATED copy of the bench, to prove the
# flow frame works when the source is not due west of the destination.
# Usage: run_rot.sh BOARD K OUT
cd "$(dirname "$0")"
python3 -u topo_emit.py --board "$1" --nets "$(python3 coherent_nets.py "$2")" \
  --out "$3"
