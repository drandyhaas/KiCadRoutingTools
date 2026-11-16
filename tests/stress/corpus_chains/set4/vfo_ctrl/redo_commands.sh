#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/vfo_ctrl
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/vfo_ctrl.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/vfo_ctrl
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.2 --track-width 0.2 --output step1b_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/vfo_ctrl
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.2 --width 0.2 --output step1b_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/vfo_ctrl
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.1 --width 0.2 --output step1b_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/vfo_ctrl
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.1 --width 0.15 --output step1b_fanout_v2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/vfo_ctrl
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanout_v2.kicad_pcb step1c_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/vfo_ctrl
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1c_capclear.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +5V --power-nets-widths 0.3 0.4 0.4 --layers F.Cu B.Cu --layer-costs 1.0 3.0
