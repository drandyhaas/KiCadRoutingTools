#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/kawaii_dock
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/kawaii_dock
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set13/kawaii_dock.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.508 --via-drill 0.254 --track-width 0.127 --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/kawaii_dock
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/kawaii_dock
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --track-width 0.127 --clearance-ceiling 0.127 --via-size 0.508 --via-drill 0.254 --hole-to-hole-clearance 0.2 --power-nets GND VBUS VKAWAII VKAWAII_SWITCHED --power-nets-widths 0.3 0.4 0.4 0.4 --max-ripup 5 --stats
