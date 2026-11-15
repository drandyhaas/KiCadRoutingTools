#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/noahfc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/noahfc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/noahfc.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.45 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/noahfc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/noahfc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u2.kicad_pcb --component U2 --nets '*' '!GND' --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/noahfc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/noahfc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u2.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/noahfc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/noahfc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_capclear.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND 5V 3.3V VCC VIN --power-nets-widths 0.4 0.5 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/noahfc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_capclear.kicad_pcb step3_route_v2.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND 5V 3.3V VCC VIN BEC_5V --power-nets-widths 0.4 0.6 0.4 0.4 1.0 1.0
