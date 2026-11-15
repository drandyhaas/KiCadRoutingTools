#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ethercat_8in8out
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set16/ethercat_8in8out.kicad_pcb step1_pour.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ethercat_8in8out
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_pour.kicad_pcb --component U3 --nets '*' '!GND' --clearance 0.1 --output step1b_fanout.kicad_pcb
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ethercat_8in8out
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanout.kicad_pcb step1c_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ethercat_8in8out
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1c_clearance.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3.3V +1V2 +3.3VA -24V_FIELD VDD12TX1 +5V +24V_FIELD --power-nets-widths 0.3 0.3 0.25 0.25 0.4 0.25 0.3 0.4 --layers F.Cu B.Cu --layer-costs 1.0 3.0
