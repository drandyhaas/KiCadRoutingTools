#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/chart_plotter_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/chart_plotter_hat.kicad_pcb step1_planes.kicad_pcb --nets GND GND2 +5V --plane-layers In1.Cu In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/chart_plotter_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND GND2 +5V +3.3V +5VP +12V VEE /12VBAT 'Net-(U1-AVCC)' --power-nets-widths 0.4 0.3 0.5 0.3 0.3 0.4 0.3 0.5 0.3 --stats
