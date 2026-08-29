#!/bin/bash
# The braid AND the plan on one picture, per checkpoint, into
# ~/Downloads/bus.
#
# The braid is real copper emitted by topo_emit -- U1 tooth, out across
# the board, toward the berth. The plan is drawn over it on the Eco/Cmts
# layers: the corridor leg it says each net should take, the escapes at
# both ends, and the nets that still have to dive. Seeing them together
# is the only way to tell whether the braid is going where the plan
# wants it to, or somewhere else entirely.
cd "$(dirname "$0")"
DEST=~/Downloads/bus
mkdir -p "$DEST"
for K in "$@"; do
  echo "=== K$K  $(date +%H:%M:%S)"
  NETS=$(python3 coherent_nets.py "$K")
  python3 -u topo_emit.py --nets "$NETS" --out "braid_k${K}" \
    > "braid_k${K}.log" 2>&1
  if [ ! -f "braid_k${K}.kicad_pcb" ]; then
    echo "  NO BRAID: $(tail -3 "braid_k${K}.log" | head -2)"
    continue
  fi
  python3 grade_k.py "braid_k${K}.kicad_pcb" "$NETS"
  python3 make_handoff.py "$DEST/take4_k${K}_braid.kicad_pcb" "$K" \
    "--on=braid_k${K}.kicad_pcb" 2>&1 | tail -1
  VIEW=$(python3 view_of.py "$K")
  python3 render_eco.py "$DEST/take4_k${K}_braid.kicad_pcb" \
    "$DEST/take4_k${K}_braid.png" --nets "$NETS" --view "$VIEW" 2>&1 | tail -1
done
echo "=== braid+plan renders done $(date +%H:%M:%S)"
