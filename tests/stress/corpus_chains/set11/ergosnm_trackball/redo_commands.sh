#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set11/ergosnm_trackball.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers F.Cu --clearance-ceiling 0.127 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -c U4 --output step2_fanout_u4.kicad_pcb --nets '*' '!GND' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --clearance 0.1 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u4.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diff.kicad_pcb --nets /USB_D+ /USB_D- --track-width 0.1 --diff-pair-gap 0.1 --impedance 90 --clearance-ceiling 0.127 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diff.kicad_pcb --nets /USB_D+ /USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.127 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +1V1 VBUS +5V --power-nets-widths 0.3 0.3 0.25 0.5 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +1V1 VBUS +5V --power-nets-widths 0.3 0.3 0.25 0.5 0.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/ergosnm_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5b_route_thin.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +1V1 VBUS +5V --power-nets-widths 0.3 0.3 0.25 0.5 0.5 --max-ripup 5
