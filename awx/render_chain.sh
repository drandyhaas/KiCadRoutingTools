#!/bin/bash
# Render the chain's boards next to the plain braid at the same K, into
# ~/Downloads/bus. The chain measured worse at every checkpoint, so
# these are the pictures of HOW it is worse -- the escapes the fanout
# laid, and the braid that then had to work with them.
#
# usage: render_chain.sh BOARD:LABEL:K [BOARD:LABEL:K ...]
cd "$(dirname "$0")"
DEST=~/Downloads/bus
mkdir -p "$DEST"
for spec in "$@"; do
  BOARD=${spec%%:*}
  rest=${spec#*:}
  LABEL=${rest%%:*}
  K=${rest##*:}
  if [ ! -f "$BOARD" ]; then
    echo "MISSING $BOARD"; continue
  fi
  NETS=$(python3 coherent_nets.py "$K")
  VIEW=$(python3 view_of.py "$K")
  python3 render_eco.py "$BOARD" "$DEST/${LABEL}.png" \
    --nets "$NETS" --view "$VIEW" 2>&1 | tail -1
  python3 grade_k.py "$BOARD" "$NETS" 2>&1 | tail -1
done
echo "=== chain renders done $(date +%H:%M:%S)"
