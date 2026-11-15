#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/mantis_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set8/mantis_kb.kicad_pcb step1_pour.kicad_pcb --nets GND GND --plane-layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/mantis_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND --power-nets-widths 0.3 --layers F.Cu B.Cu --layer-costs 3.0 3.0
