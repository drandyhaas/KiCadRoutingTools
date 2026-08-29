#!/bin/bash
# Grade + per-net audit of one emitted board at a coherent checkpoint.
# Usage: check_k.sh BOARD K
cd "$(dirname "$0")"
NETS=$(python3 coherent_nets.py "$2")
python3 grade_k.py "$1" "$NETS"
python3 audit_nets.py "$1" "$2"
