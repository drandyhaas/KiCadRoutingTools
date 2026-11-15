#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/espresso_hv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/espresso_hv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set25/espresso_hv.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --power-nets GND --power-nets-widths 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/espresso_hv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/espresso_hv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V --power-nets-widths 0.4 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/espresso_hv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V 120VAC_HOT 120VAC_NEUTRAL /120VAC_HOT_SWITCHED /BOILER_HOT_OUT /PUMP_HOT_OUT --power-nets-widths 0.4 0.4 0.5 0.5 0.5 0.5 0.5 --max-ripup 5 --overwrite
