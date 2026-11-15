#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/picofx_pump
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/picofx_pump
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/picofx_pump.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set21/picofx_pump/step1_planes.kicad_pcb --nets GND GND --plane-layers F.Cu B.Cu --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/picofx_pump
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/picofx_pump
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set21/picofx_pump/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set21/picofx_pump/step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.254 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V --power-nets-widths 0.254 0.381
