#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/tigard
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/tigard.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/tigard
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U3 --nets '*' '!GND' '!+3V3' --output step1b_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/tigard
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanout.kicad_pcb step1c_capopt.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/tigard
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1c_capopt.kicad_pcb step2a_diffpairs.kicad_pcb --nets /USB_DP /USB_DN --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/tigard
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --track-width 0.10 --clearance-ceiling 0.15 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --grid-step 0.05 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --power-nets GND +3V3 +5V VTARGET VREG /VBUS +1V8 VREF --power-nets-widths 0.3 0.3 0.3 0.25 0.25 0.3 0.25 0.2
