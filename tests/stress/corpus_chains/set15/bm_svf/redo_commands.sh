#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/bm_svf
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set15/bm_svf.kicad_pcb step1_pour.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/bm_svf
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.7 --via-drill 0.3 --power-nets GND V_SUPPLY --power-nets-widths 0.3 0.4 --layers F.Cu B.Cu --layer-costs 1.0 3.0
