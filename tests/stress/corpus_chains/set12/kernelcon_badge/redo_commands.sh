#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/kernelcon_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/kernelcon_badge.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set12/kernelcon_badge/step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/kernelcon_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set12/kernelcon_badge/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set12/kernelcon_badge/step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.2 --power-nets GND +5V +3.3V BATCRANK-VCC --power-nets-widths 0.3 0.5 0.4 0.3 --layers F.Cu B.Cu --layer-costs 1.0 3.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/kernelcon_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set12/kernelcon_badge/step2_route.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set12/kernelcon_badge/step2b_pg.kicad_pcb --nets 'unconnected-(U5-PG-Pad5)' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.2 --layers F.Cu B.Cu --layer-costs 1.0 3.0
