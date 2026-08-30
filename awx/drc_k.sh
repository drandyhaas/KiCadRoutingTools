#!/bin/bash
# The DRC pairs on a board, counted -- WHICH nets graze is the question
# a bare count never answers (K15 chain: 6 DRC, all of them the
# plane-drop pass vs the decoupling caps, none of them the braid's).
# usage: drc_k.sh BOARD
cd "$(dirname "$0")"
python3 ../py_router/check_drc.py "$1" --clearance 0.1 \
  --clearance-margin 0.1 2>&1 | grep -E "FOUND|NO DRC|<->" \
  | sed 's/ at .*//' | sort | uniq -c | sort -rn | head -20
