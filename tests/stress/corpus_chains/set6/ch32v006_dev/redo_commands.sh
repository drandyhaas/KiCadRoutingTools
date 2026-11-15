#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v006_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set6/ch32v006_dev.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v006_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2a_diff.kicad_pcb --nets D+ D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v006_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +3.3V VBUS --power-nets-widths 0.3 0.4 0.4 --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --layers F.Cu B.Cu
