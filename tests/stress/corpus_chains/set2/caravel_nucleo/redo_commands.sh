#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/caravel_nucleo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/caravel_nucleo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/caravel_nucleo.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/caravel_nucleo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.1 --output step2_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/caravel_nucleo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_fanout_u1.kicad_pcb --component U10 --nets '*' '!GND' --clearance 0.1524 --output step3_fanout_u10.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/caravel_nucleo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_fanout_u10.kicad_pcb step4_capopt.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/caravel_nucleo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/caravel_nucleo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4_capopt.kicad_pcb step5_diff.kicad_pcb --nets FTDI_D+ FTDI_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/caravel_nucleo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/caravel_nucleo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diff.kicad_pcb step6_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.1524 --via-size 0.45 --via-drill 0.2032 --hole-to-hole-clearance 0.2 --power-nets GND vdda vccd +5V vdda1 vdda2 vccd2 FTDI_V3.3 --power-nets-widths 0.3 0.4 0.4 0.5 0.3 0.3 0.3 0.3
