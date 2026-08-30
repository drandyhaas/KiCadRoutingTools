#!/bin/bash
# grade_k.py for a board at checkpoint K (nets from the coherent ladder).
# usage: grade_kb.sh BOARD K [K BOARD ...]
cd "$(dirname "$0")"
while [ $# -ge 2 ]; do
  python3 grade_k.py "$1" "$(python3 coherent_nets.py "$2")"
  shift 2
done
