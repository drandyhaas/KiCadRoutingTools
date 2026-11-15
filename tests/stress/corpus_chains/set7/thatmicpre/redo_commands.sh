#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/thatmicpre
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/thatmicpre.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/thatmicpre
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND /VIN --power-nets-widths 0.3 0.6 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --clearance-ceiling 0.25 --track-width 0.5 --via-size 0.45 --via-drill 0.3
