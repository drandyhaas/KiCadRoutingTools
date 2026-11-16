#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/squishbox
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set11/squishbox.kicad_pcb squishbox_step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/squishbox
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py squishbox_step1_planes.kicad_pcb squishbox_step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V /5.2V /VIN --power-nets-widths 0.4 0.4 0.4 0.4 0.4 --max-ripup 5
