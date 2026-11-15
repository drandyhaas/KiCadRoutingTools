#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/varner_sipm_revb
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/varner_sipm_revb
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/varner_sipm_revb.kicad_pcb step1_planes.kicad_pcb --nets GND GND --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/varner_sipm_revb
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/varner_sipm_revb
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3 --track-width 0.25 --power-nets GND '+BATT 1' '+BATT 2' '+BATT 3' '+BATT 4' --power-nets-widths 0.4 0.4 0.4 0.4 0.4 --max-ripup 5
