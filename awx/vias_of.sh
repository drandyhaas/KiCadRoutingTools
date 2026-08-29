#!/bin/bash
# Via census of a ladder rung board. Usage: vias_of.sh BOARD K
cd "$(dirname "$0")"
python3 via_census.py "$1" "$(python3 ladder_nets.py "$2")"
