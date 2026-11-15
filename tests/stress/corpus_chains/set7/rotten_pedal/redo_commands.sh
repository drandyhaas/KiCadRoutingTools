#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rotten_pedal
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rotten_pedal
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/rotten_pedal.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rotten_pedal
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.4 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +9V /+4V5 /+9V_IN --power-nets-widths 0.6 0.6 0.6 0.6
