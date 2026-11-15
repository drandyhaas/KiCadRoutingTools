#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/in13_nixie
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set17/in13_nixie.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/in13_nixie
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb --nets D+ D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/in13_nixie
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND +3.3V VBUS HV --power-nets-widths 0.4 0.4 0.5 0.5 --layer-costs 1.0 3.0
