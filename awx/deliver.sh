#!/bin/bash
# Deliver a board + render to ~/Downloads/bus for visual verification.
# Usage: deliver.sh BOARD K NAME [view]
cd "$(dirname "$0")"
BOARD=$1; K=$2; NAME=$3; VIEW=${4:-118,56,150,76}
DEST=~/Downloads/bus
mkdir -p "$DEST"
python3 ../py_router/copy_board.py "$BOARD" "$DEST/$NAME.kicad_pcb" \
  >/dev/null 2>&1 || cp "$BOARD" "$DEST/$NAME.kicad_pcb"
[ -f "${BOARD%.kicad_pcb}.kicad_pro" ] && \
  cp "${BOARD%.kicad_pcb}.kicad_pro" "$DEST/$NAME.kicad_pro"
python3 render_eco.py "$BOARD" "$DEST/$NAME.png" \
  --nets "$(python3 coherent_nets.py "$K")" --view "$VIEW"
echo "delivered $DEST/$NAME.{kicad_pcb,png}"
