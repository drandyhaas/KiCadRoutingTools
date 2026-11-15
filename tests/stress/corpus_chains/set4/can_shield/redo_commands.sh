#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/can_shield
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/can_shield
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/can_shield.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --power-nets GND --power-nets-widths 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/can_shield
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/can_shield
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +8V +24V +5V +3V3 --power-nets-widths 0.4 0.4 0.4 0.4 0.3
