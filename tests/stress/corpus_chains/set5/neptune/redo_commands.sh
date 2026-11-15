#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/neptune
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/neptune.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set5/neptune/step1_planes.kicad_pcb --nets GND +8VREF --plane-layers In1.Cu In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/neptune
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set5/neptune/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set5/neptune/step2_route.kicad_pcb --nets '*' --power-nets GND +8VREF -12V +12V -8VREF --power-nets-widths 0.3 0.3 0.3 0.3 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 6.0 2.5 1.0 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.5
