#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/robocup_kicker
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/robocup_kicker
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/robocup_kicker
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/robocup_kicker
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/robocup_kicker
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/robocup_kicker.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.254 --via-size 1.3 --via-drill 0.65 --track-width 0.254 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/robocup_kicker
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.3 --track-width 0.3 --via-size 1.3 --via-drill 0.65 --hole-to-hole-clearance 0.2 --power-nets GND +24V 5V +45V +3V3 'Net-(D1-K)' 'Net-(J7-Pin_2)' --power-nets-widths 0.6 2.6 0.6 3.0 0.4 3.0 3.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/robocup_kicker
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
