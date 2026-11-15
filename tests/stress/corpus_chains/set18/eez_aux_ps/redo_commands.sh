#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/eez_aux_ps
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/eez_aux_ps.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/eez_aux_ps
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.1016 --track-width 0.254 --via-size 0.5 --via-drill 0.3 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --power-nets GND PE +3V3 +5V +12V AC_N_OUT AC_L_OUT '*AC_N_IN' '*AC_L_IN' '*FAN_OUT+' '*FAN_OUT-' --power-nets-widths 0.4 0.6 0.4 0.4 0.4 1.0 1.0 1.0 1.0 0.4 0.4 --max-ripup 5
