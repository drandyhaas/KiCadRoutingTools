#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ir_irradiance_probe
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ir_irradiance_probe
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/ir_irradiance_probe.kicad_pcb step1_pour.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.125
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ir_irradiance_probe
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.125 --track-width 0.15 --via-size 0.5 --via-drill 0.25 --power-nets GND +3.3VA /3V3_JST /5V_FILTERED /5V_PWR_SUPPLY /5V_USB 'Net-(C4-Pad2)' --power-nets-widths 0.3 0.25 0.25 0.25 0.25 0.25 0.25 --layer-costs 1.0 3.0 --no-bga-zone --max-ripup 5
