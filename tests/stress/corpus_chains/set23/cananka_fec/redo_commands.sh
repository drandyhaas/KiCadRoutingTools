#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cananka_fec
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set23/cananka_fec.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cananka_fec
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb --pairs USB_DP USB_DM --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.15 --via-size 0.6 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cananka_fec
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb --nets USB_DP USB_DM --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.15 --via-size 0.6 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cananka_fec
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.2 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND 5V USB_Vin 5V_SW VDD_CORE 3V3 --power-nets-widths 0.3 0.4 0.4 0.4 0.3 0.3
