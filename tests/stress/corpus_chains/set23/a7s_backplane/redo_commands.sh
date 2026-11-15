#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/a7s_backplane
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set23/a7s_backplane.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/a7s_backplane
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND +3V3 +5V +5V_IN +3V3_IN --power-nets-widths 0.3 0.4 0.4 0.4 0.4 --layers F.Cu B.Cu --layer-costs 1.0 3.0
