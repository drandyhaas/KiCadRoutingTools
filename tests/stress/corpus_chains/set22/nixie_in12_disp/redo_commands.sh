#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/nixie_in12_disp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/nixie_in12_disp.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/nixie_in12_disp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step1_planes.kicad_pcb step2_hv.kicad_pcb --nets VCC 'Net-(NE1-Pad2)' 'Net-(NE2-Pad2)' 'Net-(R13-Pad1)' 'Net-(R14-Pad1)' 'Net-(R15-Pad1)' 'Net-(R16-Pad1)' --clearance-ceiling 0.75 --track-width 0.25 --via-size 0.45 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/nixie_in12_disp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_hv.kicad_pcb step3_signal.kicad_pcb --nets '*' '!VCC' '!Net-(NE1-Pad2)' '!Net-(NE2-Pad2)' '!Net-(R13-Pad1)' '!Net-(R14-Pad1)' '!Net-(R15-Pad1)' '!Net-(R16-Pad1)' --clearance-ceiling 0.3 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND +5V --power-nets-widths 0.4 0.4
