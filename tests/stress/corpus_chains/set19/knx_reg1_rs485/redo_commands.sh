#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/knx_reg1_rs485
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/knx_reg1_rs485
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set19/knx_reg1_rs485.kicad_pcb step1_planes.kicad_pcb --nets GND GND1 --plane-layers B.Cu B.Cu --via-size 0.45 --via-drill 0.3 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --power-nets +5VP +3.3V +5V GND GND1 --power-nets-widths 0.3 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/knx_reg1_rs485
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.16 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets +5VP +3.3V +5V GND GND1 --power-nets-widths 0.3 0.3 0.3 0.3 0.3 --max-ripup 5
