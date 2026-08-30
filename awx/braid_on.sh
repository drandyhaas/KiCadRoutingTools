#!/bin/bash
# The braid stage of the chain alone: braid from the source teeth to
# the destination's stub ends on an ALREADY fanned-out board, grade,
# and (optionally) render. Lets the braid be iterated without paying
# for the fanout each time.
#
# usage: braid_on.sh FO_BOARD OUT_TAG K [render]
cd "$(dirname "$0")"
FO=$1
OUT=$2
K=$3
NETS=$(python3 coherent_nets.py "$K")
python3 -u braid.py --board "$FO" --dest "${DEST:-DU1}" --nets "$NETS" \
  --out "$OUT" > "$OUT.log" 2>&1
if [ -f "$OUT.kicad_pcb" ]; then
  grep -E "south river|launch order|entry order|divers|swaps in|WARNING|octilinearized|violations|CLEAN|wrote" "$OUT.log" | cut -c1-200
  python3 grade_k.py "$OUT.kicad_pcb" "$NETS"
  if [ "$4" = "render" ]; then
    VIEW=$(python3 view_of.py "$K")
    mkdir -p ~/Downloads/bus
    python3 render_eco.py "$OUT.kicad_pcb" ~/Downloads/bus/"$OUT.png" \
      --nets "$NETS" --view "$VIEW" 2>&1 | tail -1
  fi
else
  echo "NO BRAID:"
  grep -E 'Error|assert|Traceback' -A3 "$OUT.log" | tail -6
fi
