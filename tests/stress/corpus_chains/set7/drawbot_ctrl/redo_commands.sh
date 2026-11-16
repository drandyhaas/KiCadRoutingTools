#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/drawbot_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/drawbot_ctrl.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/drawbot_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb --pairs X_A+:X_A- X_B+:X_B- Y_A+:Y_A- Y_B+:Y_B- Z_A+:Z_A- Z_B+:Z_B- --track-width 1.0 --diff-pair-gap 0.25 --clearance-ceiling 0.25 --via-size 1.2 --via-drill 0.6 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/drawbot_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/drawbot_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb X_A+ X_A- X_B+ X_B- Y_A+ Y_A- Y_B+ Y_B- Z_A+ Z_A- Z_B+ Z_B- --track-width 1.0 --diff-pair-gap 0.25 --clearance-ceiling 0.25 --via-size 1.2 --via-drill 0.6 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/drawbot_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND +3.3V +5V +5V_ESTOP +MOTOR_PWR +LASER_PWR --power-nets-widths 0.5 0.8 0.8 0.8 1.5 1.5 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/drawbot_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_route.kicad_pcb step3b_retry33v.kicad_pcb --nets +3.3V --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --power-nets +3.3V --power-nets-widths 0.5 --max-ripup 5 --grid-step 0.1 --hole-to-hole-clearance 0.2
