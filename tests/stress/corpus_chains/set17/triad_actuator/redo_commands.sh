#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/triad_actuator
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/triad_actuator
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set17/triad_actuator.kicad_pcb step1_pour.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/triad_actuator
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_pour.kicad_pcb --component U3 --nets '*' '!GND' --clearance 0.15 --width 0.15 --grid-step 0.1 --output step1b_u3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/triad_actuator
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_u3.kicad_pcb --component U2 --nets '*' '!GND' --clearance 0.15 --width 0.15 --grid-step 0.1 --output step1b_u2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/triad_actuator
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_u2.kicad_pcb step1c.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/triad_actuator
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/triad_actuator
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1c.kicad_pcb --nets D+ D- --output step2a_diff.kicad_pcb --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/triad_actuator
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND VIN 3V3 VBUS PH_A PH_B PH_C --power-nets-widths 0.3 0.4 0.3 0.3 0.3 0.3 0.3 --clearance-ceiling 0.15 --track-width 0.15 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --layers F.Cu B.Cu --layer-costs 1.0 3.0
