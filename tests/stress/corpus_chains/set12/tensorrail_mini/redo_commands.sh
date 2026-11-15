#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/tensorrail_mini.kicad_pcb --component U1 --output step1_fanout_u1.kicad_pcb --clearance 0.15 --width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_u1.kicad_pcb --component U2 --output step2_fanout_u2.kicad_pcb --clearance 0.15 --width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u2.kicad_pcb step3_capclear.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step3_capclear.kicad_pcb step4_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tensorrail_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb '*' --clearance-ceiling 0.15 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 VBUS_RAW +1V2 --power-nets-widths 0.4 0.5 0.5 0.3 --stats
