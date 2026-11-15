#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/bornhack_circle_badge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/bornhack_circle_badge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set24/bornhack_circle_badge.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/bornhack_circle_badge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb --pairs /esp31-c3-mini/USB_D+ /esp31-c3-mini/USB_D- --track-width 0.2 --diff-pair-gap 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/bornhack_circle_badge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb '*USB_D+*' '*USB_D-*' --track-width 0.2 --diff-pair-gap 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/bornhack_circle_badge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/bornhack_circle_badge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VCC '/PSU 2xAA USB 3v3/USB_BUS_POWER' --power-nets-widths 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/bornhack_circle_badge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VCC '/PSU 2xAA USB 3v3/USB_BUS_POWER' --power-nets-widths 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/bornhack_circle_badge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VCC '/PSU 2xAA USB 3v3/USB_BUS_POWER' --power-nets-widths 0.4 0.4 0.4
