#!/bin/bash
# DRC count of a board at the routed floor, plus its first pairs.
# usage: drc_of.sh BOARD [N]
cd "$(dirname "$0")"
python3 ../py_router/check_drc.py "$1" --clearance 0.1 --clearance-margin 0.1 \
  2>&1 | grep -E "FOUND|NO DRC|<->" | head -"${2:-8}"
