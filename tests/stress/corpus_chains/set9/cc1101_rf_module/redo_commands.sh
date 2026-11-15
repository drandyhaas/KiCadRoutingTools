#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/cc1101_rf_module
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/cc1101_rf_module.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/cc1101_rf_module
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component U1 --nets '*' '!GND' --width 0.15 --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/cc1101_rf_module
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step2b_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/cc1101_rf_module
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_clearance.kicad_pcb step3_route_u1.kicad_pcb --nets /CS /GDO0 /GDO2 /MISO /MOSI /SCK 'Net-(C51-Pad1)' 'Net-(L121-Pad2)' 'Net-(L131-Pad1)' 'Net-(R171-Pad2)' 'Net-(U1-Pad10)' 'Net-(U1-Pad8)' VPP --clearance-ceiling 0.15 --track-width 0.15 --grid-step 0.05 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets VPP --power-nets-widths 0.3 --max-ripup 5 --no-bga-zones
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/cc1101_rf_module
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_route_u1.kicad_pcb step4_route_all.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --grid-step 0.1 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VPP --power-nets-widths 0.4 0.3 --max-ripup 5 --no-bga-zones
