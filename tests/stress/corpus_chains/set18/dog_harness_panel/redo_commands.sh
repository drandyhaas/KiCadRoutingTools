#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/dog_harness_panel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/dog_harness_panel.kicad_pcb step1_planes.kicad_pcb --nets GND +3.3V --plane-layers B.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/dog_harness_panel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +3.3V --power-nets-widths 0.4 0.4 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --layers F.Cu B.Cu
