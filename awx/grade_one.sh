#!/bin/bash
# Grade one K-board on its coherent-ladder nets. usage: grade_one.sh BOARD K
cd "$(dirname "$0")"
python3 grade_k.py "$1" "$(python3 coherent_nets.py "$2")"
