#!/bin/bash
# Profile one braid run: cProfile over braid.py on a recorded fanout
# board, then the top of the profile by cumulative and by own time.
# usage: prof_k.sh FANOUT_BOARD K OUTSTEM
cd "$(dirname "$0")"
BOARD=$1
K=$2
OUT=$3
NETS=$(python3 coherent_nets.py "$K")
/usr/bin/time -p python3 -m cProfile -o "${OUT}.prof" braid.py --board "$BOARD" \
  --dest DU1 --nets "$NETS" --out "$OUT" > "${OUT}.log" 2> "${OUT}.time"
cat "${OUT}.time"
python3 prof_top.py "${OUT}.prof"
