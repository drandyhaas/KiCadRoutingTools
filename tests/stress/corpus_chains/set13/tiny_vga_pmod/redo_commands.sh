#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/tiny_vga_pmod
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set13/tiny_vga_pmod.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --zone-clearance 0.127
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/tiny_vga_pmod
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +3.3V --power-nets-widths 0.35 0.35 --clearance-ceiling 0.127 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2
