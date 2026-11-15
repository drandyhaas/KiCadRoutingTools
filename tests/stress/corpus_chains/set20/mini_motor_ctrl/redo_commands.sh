#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/mini_motor_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/mini_motor_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set20/mini_motor_ctrl.kicad_pcb --component U3 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --output step1_qfn_u3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/mini_motor_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_qfn_u3.kicad_pcb --component U2 --clearance 0.1 --output step2_qfn_u2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/mini_motor_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/mini_motor_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step2_qfn_u2.kicad_pcb step3_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/mini_motor_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/mini_motor_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/mini_motor_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/mini_motor_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_planes.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 VBUS +3.3VA ADC_VBUS --power-nets-widths 0.3 0.3 0.5 0.2 0.2
