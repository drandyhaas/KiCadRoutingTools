#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/openinverter_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/openinverter_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/openinverter_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/openinverter_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/openinverter_mini.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3 --track-width 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/openinverter_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step1b_fanout_ic4.kicad_pcb --component IC4 --clearance 0.1 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/openinverter_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/openinverter_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/openinverter_mini
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1b_fanout_ic4.kicad_pcb step3_route.kicad_pcb --nets '*' --power-nets GND VCCIO VCC +12V --power-nets-widths 0.3 0.3 0.3 0.5 --clearance-ceiling 0.1 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
