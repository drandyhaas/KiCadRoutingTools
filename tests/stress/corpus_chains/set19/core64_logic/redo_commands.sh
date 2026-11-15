#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/core64_logic
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set19/core64_logic.kicad_pcb step1_pour.kicad_pcb --nets -BATT --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/core64_logic
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set19/core64_logic.kicad_pcb step1_pour.kicad_pcb --nets=-BATT --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/core64_logic
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.508 --via-size 0.45 --via-drill 0.3 --power-nets -BATT 3V3 5V0 --power-nets-widths 0.508 0.508 0.508 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/core64_logic
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.508 --via-size 0.45 --via-drill 0.3 --power-nets=-BATT 3V3 5V0 --power-nets-widths 0.508 0.508 0.508 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/core64_logic
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.508 --via-size 0.45 --via-drill 0.3 --power-nets '*BATT' 3V3 5V0 --power-nets-widths 0.508 0.508 0.508 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/core64_logic
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.508 --via-size 0.45 --via-drill 0.3 --power-nets '*BATT' 3V3 5V0 --power-nets-widths 0.508 0.508 0.508 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/core64_logic
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.508 --via-size 0.45 --via-drill 0.3 --power-nets '*BATT' 3V3 5V0 --power-nets-widths 0.508 0.508 0.508 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/core64_logic
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.508 --via-size 0.45 --via-drill 0.3 --power-nets '*BATT' 3V3 5V0 --power-nets-widths 0.508 0.508 0.508 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/core64_logic
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.508 --via-size 0.45 --via-drill 0.3 --power-nets '*BATT' 3V3 5V0 --power-nets-widths 0.508 0.508 0.508 --max-ripup 5
