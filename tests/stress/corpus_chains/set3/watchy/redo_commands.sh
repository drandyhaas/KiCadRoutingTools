#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/watchy.kicad_pcb --output step1_fanout_u4.kicad_pcb --component U4 --nets '!GND' '!+3V3' --clearance 0.09 --width 0.15 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_fanout_u4.kicad_pcb step1b_capclear_u4.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_capclear_u4.kicad_pcb --output step2_fanout_u6.kicad_pcb --component U6 --nets '!GND' '!+3V3' --clearance 0.09 --width 0.15 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u6.kicad_pcb step2b_capclear_u6.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_capclear_u6.kicad_pcb --output step3_diffpairs.kicad_pcb --nets DP DN USB_D+ USB_D- --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step3_diffpairs.kicad_pcb step4_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.5 --via-drill 0.3 --track-width 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 VBUS VDD --power-nets-widths 0.3 0.3 0.3 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/watchy
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 VBUS VDD --power-nets-widths 0.3 0.3 0.3 0.2 --stats
