#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/bread_dcmotor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/bread_dcmotor.kicad_pcb step1_planes.kicad_pcb --nets GND GNDPWR --plane-layers B.Cu B.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/bread_dcmotor
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND GNDPWR +12V +5V '/Motor Driver LMD185001/VS' '/Motor Driver LMD185002/VS' '/Motor Driver LMD185001/OUTPUT1' '/Motor Driver LMD185001/OUTPUT2' '/Motor Driver LMD185002/OUTPUT1' '/Motor Driver LMD185002/OUTPUT2' --power-nets-widths 0.6 0.6 1.2 0.6 1.2 1.2 1.2 1.2 1.2 1.2
