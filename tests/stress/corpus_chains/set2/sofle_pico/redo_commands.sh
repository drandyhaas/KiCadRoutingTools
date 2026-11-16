#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/sofle_pico
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/sofle_pico
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/sofle_pico.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.45 --via-drill 0.2 --track-width 0.25 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.3 --power-nets VCC --power-nets-widths 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/sofle_pico
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/sofle_pico
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets VCC --power-nets-widths 0.5 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/sofle_pico
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets VCC --power-nets-widths 0.5 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.3 --max-ripup 5
