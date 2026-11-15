#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/picoprobe_tester
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/picoprobe_tester.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/picoprobe_tester
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +5V /VBus --power-nets-widths 0.5 0.4 0.4 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/picoprobe_tester
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route_v2.kicad_pcb --nets '*' --power-nets GND +5V /VBus --power-nets-widths 0.5 0.4 0.4 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5 --layer-costs 1.0 1.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/picoprobe_tester
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/picoprobe_tester.kicad_pcb step1_route_v3.kicad_pcb --nets '*' '!GND' --power-nets +5V /VBus --power-nets-widths 0.4 0.4 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5 --layer-costs 1.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/picoprobe_tester
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step1_route_v3.kicad_pcb step2_route_v3.kicad_pcb --nets GND --plane-layers B.Cu
