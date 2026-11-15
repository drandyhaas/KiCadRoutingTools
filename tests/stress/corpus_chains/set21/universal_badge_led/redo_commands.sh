#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/universal_badge_led
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/universal_badge_led.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/universal_badge_led
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step1_planes.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND 3V3 --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/universal_badge_led
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_route.kicad_pcb step4_retry.kicad_pcb --nets /PWM7R --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --max-ripup 5 --no-bga-zone
