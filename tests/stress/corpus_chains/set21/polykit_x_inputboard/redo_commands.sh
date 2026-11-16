#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/polykit_x_inputboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/polykit_x_inputboard.kicad_pcb board_step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/polykit_x_inputboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py board_step1_planes.kicad_pcb board_step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --power-nets GND +5V +12V -12V -5V --power-nets-widths 0.3 0.4 0.4 0.4 0.4 --layers F.Cu B.Cu --layer-costs 1.0 3.0
