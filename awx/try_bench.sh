#!/bin/bash
# Try destination offsets for the 45-degree bench until the moved part
# collides with nothing (the caps under it on the base bench).
# usage: try_bench.sh "dx,dy dx,dy ..."
cd "$(dirname "$0")"
for XY in $1; do
  DX=${XY%,*}
  DY=${XY#*,}
  python3 make_bench.py fb_t2q_base.kicad_pcb /tmp/claude-501/try_d45.kicad_pcb \
    DU1 --rot 45 --dx "$DX" --dy "$DY" > /dev/null || continue
  N=$(python3 ../py_router/check_drc.py /tmp/claude-501/try_d45.kicad_pcb \
    --clearance 0.1 --clearance-margin 0.1 2>&1 | grep -c "<->")
  echo "dx=$DX dy=$DY: $N pad conflicts"
done
