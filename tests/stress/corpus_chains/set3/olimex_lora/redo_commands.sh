#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/olimex_lora
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/olimex_lora
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/olimex_lora.kicad_pcb --output step1_qfn_fanout.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.2 --width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/olimex_lora
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_qfn_fanout.kicad_pcb step2_clearance.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/olimex_lora
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/olimex_lora
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step2_clearance.kicad_pcb step3_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.45 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --power-nets GND --power-nets-widths 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/olimex_lora
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/olimex_lora
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_planes.kicad_pcb step4_route_u1.kicad_pcb --component U1 --grid-step 0.05 --clearance-ceiling 0.15 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/olimex_lora
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_route_u1.kicad_pcb step5_final.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VAA --power-nets-widths 0.4 0.3
