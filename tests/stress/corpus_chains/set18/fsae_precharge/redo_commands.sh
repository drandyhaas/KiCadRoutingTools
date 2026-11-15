#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_precharge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/fsae_precharge.kicad_pcb step1_pour.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_precharge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V /+5V_TS_FUSED /VCC /HV_BAT+ /HV_TS+ /HV_TS- /HV_TS+_Fuse /TS+_FUSED 'Net-(R3-Pad1)' 'Net-(K1-COM_(+))' 'Net-(K1-COM)' 'Net-(K3-COM_(+))' 'Net-(K3-COM)' --power-nets-widths 1.0 0.4 0.4 0.4 1.5 0.6 1.0 0.6 0.4 1.5 1.5 1.5 1.5 1.5 --layer-costs 1.0 3.0
