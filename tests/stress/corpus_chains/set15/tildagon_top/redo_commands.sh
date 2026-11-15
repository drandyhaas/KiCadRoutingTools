#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_top
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_top
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_top
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set15/tildagon_top.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.16 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_top
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.16 --track-width 0.2 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND /VBAT_SW /3V3 --power-nets-widths 0.3 0.4 0.3 --max-ripup 5
