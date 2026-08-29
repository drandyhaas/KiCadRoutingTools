#!/bin/bash
# Grade a board emitted onto a rotated bench copy. Usage: grade_rot.sh BOARD K
cd "$(dirname "$0")"
python3 grade_k.py "$1" "$(python3 coherent_nets.py "$2")"
