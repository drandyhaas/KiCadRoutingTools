#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/kivu12
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/kivu12
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/kivu12.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.6858 --via-drill 0.3048 --track-width 0.1524 --clearance-ceiling 0.1524 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/kivu12
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/kivu12
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.1524 --via-size 0.6858 --via-drill 0.3048 --power-nets GND +3V3 --power-nets-widths 0.3048 0.3048 --hole-to-hole-clearance 0.2
