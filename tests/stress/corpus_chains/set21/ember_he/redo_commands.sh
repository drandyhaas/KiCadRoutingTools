#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/ember_he
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/ember_he
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/ember_he.kicad_pcb --component MCU1 --nets '*' '!GND' --width 0.2 --clearance 0.2 --output step1_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/ember_he
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step1_fanout.kicad_pcb step1c.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/ember_he
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/ember_he
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1c.kicad_pcb step2_diff.kicad_pcb --nets USB_D+ USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/ember_he
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/ember_he
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --no-bga-zone MCU1 --max-ripup 5 --power-nets GND +3.3V +5V --power-nets-widths 0.3 0.3 0.4 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/ember_he
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py step3_route.kicad_pcb step4_planes.kicad_pcb --nets GND GND --plane-layers F.Cu B.Cu --zone-clearance 0.2
