#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/timiq_rf
KICAD_ROUTE_TRACE=1 python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/timiq_rf
KICAD_ROUTE_TRACE=1 python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set28/timiq_rf.kicad_pcb step1_pour.kicad_pcb --nets GND 5V_2 3V3_2 --plane-layers In1.Cu In3.Cu In4.Cu --via-size 0.45 --via-drill 0.2 --clearance 0.09 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/timiq_rf
KICAD_ROUTE_TRACE=1 python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zones --max-ripup 5 --track-width 0.15 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND 5V_2 3V3_2 5V_1 3V3_1 24V -24V 15V -15V 7V5 -5V --power-nets-widths 0.3 0.4 0.3 0.4 0.3 0.4 0.4 0.3 0.3 0.3 0.3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --layer-costs 1.0 6.0 1.0 2.5 2.5 1.0 1.0 1.0
