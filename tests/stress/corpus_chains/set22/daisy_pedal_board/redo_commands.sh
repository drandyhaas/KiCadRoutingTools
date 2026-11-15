#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/daisy_pedal_board
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/daisy_pedal_board
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/daisy_pedal_board
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/daisy_pedal_board.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/daisy_pedal_board
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1_planes.kicad_pcb step2_diffpairs.kicad_pcb /USB-D+ /USB-D- /USBCONN-D+ /USBCONN-D- --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --impedance 90 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/daisy_pedal_board
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/daisy_pedal_board
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_diffpairs.kicad_pcb step3_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 VBUS --power-nets-widths 0.375 0.375 0.375
