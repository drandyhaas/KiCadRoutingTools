#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spimux
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spimux
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spimux
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spimux
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set16/spimux.kicad_pcb step1_planes.kicad_pcb --nets GNDREF --plane-layers B.Cu --clearance-ceiling 0.153 --via-size 0.5 --via-drill 0.2 --track-width 0.16 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spimux
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u4.kicad_pcb --component U4 --nets '*' '!GNDREF' --layers F.Cu B.Cu --clearance 0.1 --via-size 0.5 --via-drill 0.2 --track-width 0.16
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spimux
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u4.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spimux
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_capclear.kicad_pcb step3_route.kicad_pcb '*' --clearance-ceiling 0.153 --track-width 0.16 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GNDREF +1V8 +3V3 --power-nets-widths 0.3 0.3 0.3 --stats
