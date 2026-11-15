#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/rc2014_ym2149
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set11/rc2014_ym2149.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/rc2014_ym2149
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2b_route_balanced.kicad_pcb --nets '*' --clearance-ceiling 0.254 --track-width 0.381 --via-size 0.8 --via-drill 0.45 --power-nets GND VCC --power-nets-widths 0.5 0.5 --layer-costs 1.0 1.0 --max-ripup 5
