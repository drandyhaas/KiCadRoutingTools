#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/hexapod_legbrd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/hexapod_legbrd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/hexapod_legbrd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/hexapod_legbrd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set17/hexapod_legbrd.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.15 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/hexapod_legbrd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb COXA_SP COXA_SN FEMUR_SP FEMUR_SN ITOT_SP ITOT_SN TIBIA_SP TIBIA_SN --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/hexapod_legbrd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/hexapod_legbrd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets GND +6V +6V_IN +6V_COXA +6V_TIBIA +6V_FEMUR +3.3V +5V --power-nets-widths 0.8 1.4 1.4 0.8 0.8 0.8 0.4 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/hexapod_legbrd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/hexapod_legbrd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_route.kicad_pcb step3b_fix_coxa.kicad_pcb --nets +6V_COXA --force-reroute --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets +6V_COXA --power-nets-widths 0.8
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/hexapod_legbrd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_route.kicad_pcb step3b_fix_coxa.kicad_pcb --nets +6V_COXA --force-reroute --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets +6V_COXA --power-nets-widths 0.8
