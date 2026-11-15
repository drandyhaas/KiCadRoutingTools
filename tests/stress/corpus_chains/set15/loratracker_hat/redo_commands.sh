#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set15/loratracker_hat.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --output step2a_fanout_u1.kicad_pcb --clearance 0.2 --width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u1.kicad_pcb --component U3 --nets '*' '!GND' '!+3V3' --output step2b_fanout_u3.kicad_pcb --clearance 0.2 --width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u3.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2c_capclear.kicad_pcb step3a_route_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.10 --grid-step 0.05 --track-width 0.20 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3a_route_u1.kicad_pcb step3b_route_u3.kicad_pcb --component U3 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --grid-step 0.05 --track-width 0.20 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/loratracker_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3a_route_u1.kicad_pcb step3c_route_all.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --grid-step 0.05 --track-width 0.20 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 --power-nets-widths 0.25 0.25 --max-ripup 5
