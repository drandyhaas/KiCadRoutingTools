#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/mydewcontroller
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/mydewcontroller.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --zone-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/mydewcontroller
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND +5V +12V +8V --power-nets-widths 1.0 1.0 2.0 1.0
