#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/sx1262_re_433
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set24/sx1262_re_433.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set24/sx1262_re_433/step1_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2
