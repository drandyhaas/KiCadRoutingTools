#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v303_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set6/ch32v303_dev.kicad_pcb step1_planes.kicad_pcb --nets GNDD --plane-layers B.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v303_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout.kicad_pcb --component U1 --nets '*' '!GNDD' --clearance 0.2 --track-width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v303_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component U1 --nets '*' '!GNDD' --clearance 0.2 --width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v303_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v303_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v303_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v303_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_capclear.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GNDD VCC +5V --power-nets-widths 0.5 0.5 0.5
