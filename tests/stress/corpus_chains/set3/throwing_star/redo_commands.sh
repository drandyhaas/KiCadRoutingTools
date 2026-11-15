#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/throwing_star
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/throwing_star.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set3/throwing_star/step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.1778
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/throwing_star
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set3/throwing_star/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set3/throwing_star/step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.1778 --track-width 0.4064 --via-size 0.8128 --via-drill 0.508 --hole-to-hole-clearance 0.25 --power-nets GND --power-nets-widths 0.4064
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/throwing_star
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set3/throwing_star/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set3/throwing_star/step2b_route_balanced.kicad_pcb --nets '*' --clearance-ceiling 0.1778 --track-width 0.4064 --via-size 0.8128 --via-drill 0.508 --hole-to-hole-clearance 0.25 --power-nets GND --power-nets-widths 0.4064 --layer-costs 1.0 1.5
