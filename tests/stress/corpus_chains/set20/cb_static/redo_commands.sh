#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cb_static
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set20/cb_static.kicad_pcb step1_pour.kicad_pcb --nets GND GND --plane-layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cb_static
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_pour.kicad_pcb step2a_diff.kicad_pcb --nets D+ D- --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.15 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cb_static
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --power-nets GND +5V --power-nets-widths 0.3 0.3 --clearance-ceiling 0.15 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --layer-costs 1.0 1.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cb_static
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --power-nets GND +5V --power-nets-widths 0.3 0.3 --clearance-ceiling 0.15 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --layer-costs 1.0 1.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cb_static
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --power-nets GND +5V --power-nets-widths 0.3 0.3 --clearance-ceiling 0.15 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --layer-costs 1.0 1.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cb_static
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --power-nets GND +5V --power-nets-widths 0.3 0.3 --clearance-ceiling 0.15 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --layer-costs 1.0 1.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cb_static
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --power-nets GND +5V --power-nets-widths 0.3 0.3 --clearance-ceiling 0.15 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --layer-costs 1.0 1.0 --max-ripup 5
