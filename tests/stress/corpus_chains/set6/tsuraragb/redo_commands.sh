#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set6/tsuraragb.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.45 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.1 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.1 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u1.kicad_pcb --output step2b_fanout_ue.kicad_pcb --component Ue --nets '*' '!GND' --clearance 0.1 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_ue.kicad_pcb --output step2c_fanout_u3.kicad_pcb --component U3 --nets '*' '!GND' --clearance 0.1 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2c_fanout_u3.kicad_pcb step2d_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2d_capclear.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --net-clearances net_clearances_finepitch.json --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 +5V --power-nets-widths 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/tsuraragb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2d_capclear.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --net-clearances net_clearances_finepitch.json --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 +5V --power-nets-widths 0.4 0.4 0.4
