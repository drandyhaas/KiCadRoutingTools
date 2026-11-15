#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/hhkb_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set8/hhkb_ctrl.kicad_pcb step1_diffpair.kicad_pcb --pairs /D+,/D- --track-width 0.2 --diff-pair-gap 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/hhkb_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set8/hhkb_ctrl.kicad_pcb step1_diffpair.kicad_pcb /D+ /D- --track-width 0.2 --diff-pair-gap 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/hhkb_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/hhkb_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step1_diffpair.kicad_pcb step2_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.889 --via-drill 0.508 --clearance-ceiling 0.254 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/hhkb_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/hhkb_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_planes.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.254 --track-width 0.254 --via-size 0.889 --via-drill 0.508 --hole-to-hole-clearance 0.2 --power-nets GND 5V_BOOST +3.3V VUSB --power-nets-widths 0.4 0.4 0.4 0.3
