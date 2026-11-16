#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/polykit_x
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/polykit_x.kicad_pcb board_step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/polykit_x
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board_step1_planes.kicad_pcb board_step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND +5V -12V +12V -5V --power-nets-widths 0.3 0.4 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/polykit_x
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board_step1_planes.kicad_pcb board_step2b_route_balanced.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --layer-costs 1.0 1.5 --power-nets GND +5V -12V +12V -5V --power-nets-widths 0.3 0.4 0.4 0.4 0.4
