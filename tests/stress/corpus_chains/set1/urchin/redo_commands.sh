#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/urchin
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/urchin
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/urchin
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set1/urchin.kicad_pcb urchin_step1_planes.kicad_pcb --nets GND R_GND --plane-layers B.Cu B.Cu --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/urchin
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py urchin_step1_planes.kicad_pcb urchin_step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets GND R_GND +3V3 R_3V3 BT+ R_BT+ RAW R_RAW --power-nets-widths 0.3 0.3 0.25 0.25 0.25 0.25 0.25 0.25
