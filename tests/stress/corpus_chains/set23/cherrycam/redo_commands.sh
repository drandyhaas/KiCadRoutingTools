#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/private/tmp/wt_baseline_0809/py_router
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cherrycam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cherrycam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set23/cherrycam.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cherrycam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1_planes.kicad_pcb step2_diff_cam.kicad_pcb 'CAM1_*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.1 --diff-pair-gap 0.1 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cherrycam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2_diff_cam.kicad_pcb step2_diff_usb.kicad_pcb 'USB_*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.1 --diff-pair-gap 0.1 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/private/tmp/wt_baseline_0809/py_router
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cherrycam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_diff_usb.kicad_pcb step3_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 +1V8 +2V8 --power-nets-widths 0.3 0.3 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cherrycam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_diff_usb.kicad_pcb step3_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 +1V8 +2V8 --power-nets-widths 0.3 0.3 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cherrycam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_diff_usb.kicad_pcb step3_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 +1V8 +2V8 --power-nets-widths 0.3 0.3 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/cherrycam
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_diff_usb.kicad_pcb step3_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 +1V8 +2V8 --power-nets-widths 0.3 0.3 0.25 0.25
