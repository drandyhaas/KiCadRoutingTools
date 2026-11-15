#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/atf1502_cpld
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/atf1502_cpld
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/atf1502_cpld
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set6/atf1502_cpld.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/atf1502_cpld
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout.kicad_pcb --component U1 --nets '*' '!GND' --width 0.15 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/atf1502_cpld
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component U1 --nets '*' '!GND' --width 0.15 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/atf1502_cpld
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step3_declear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/atf1502_cpld
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/atf1502_cpld
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/atf1502_cpld
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_declear.kicad_pcb step4_route.kicad_pcb --nets '*' --power-nets GND +5V --power-nets-widths 0.5 0.5 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25
