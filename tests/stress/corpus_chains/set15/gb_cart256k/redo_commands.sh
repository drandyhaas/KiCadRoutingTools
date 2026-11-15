#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/gb_cart256k
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/gb_cart256k
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/gb_cart256k
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set15/gb_cart256k.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/gb_cart256k
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.1 --width 0.15 --grid-step 0.1 --output step1b_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/gb_cart256k
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.1 --width 0.1 --grid-step 0.05 --output step1b_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/gb_cart256k
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanout_u1.kicad_pcb step1c_capopt.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/gb_cart256k
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/gb_cart256k
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1c_capopt.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --clearance-ceiling 0.2 --track-width 0.32 --via-size 0.5 --via-drill 0.3 --power-nets GND +1V8 +3V3 +5V --power-nets-widths 0.3 0.4 0.4 0.4 --layers F.Cu B.Cu --layer-costs 1.0 3.0
