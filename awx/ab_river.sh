#!/bin/bash
# Is the separate 'south river' builder earning its 87 lines? Same
# checkpoints, same commit, river split ON and OFF.
cd "$(dirname "$0")"
for K in "$@"; do
  NETS=$(python3 coherent_nets.py "$K")
  for MODE in on off; do
    TAG="riv${MODE}_k${K}"
    if [ "$MODE" = off ]; then
      KICAD_NO_RIVER=1 python3 -u topo_emit.py --nets "$NETS" \
        --out "$TAG" > "$TAG.log" 2>&1
    else
      python3 -u topo_emit.py --nets "$NETS" --out "$TAG" \
        > "$TAG.log" 2>&1
    fi
    if [ ! -f "$TAG.kicad_pcb" ]; then
      echo "  river=$MODE K$K  NO BOARD: $(grep -E 'Error|assert' "$TAG.log" | tail -1 | cut -c1-80)"
      continue
    fi
    echo -n "  river=$MODE "
    python3 grade_k.py "$TAG.kicad_pcb" "$NETS"
  done
done
echo "=== river ab done"
