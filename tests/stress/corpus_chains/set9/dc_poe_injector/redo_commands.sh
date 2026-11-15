#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/dc_poe_injector
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/dc_poe_injector.kicad_pcb step1_planes.kicad_pcb --nets GND POE_45 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/dc_poe_injector
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout_u2.kicad_pcb --component U2 --clearance 0.2 --width 0.2 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/dc_poe_injector
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u2.kicad_pcb --component U2 --clearance 0.2 --width 0.2 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/dc_poe_injector
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/dc_poe_injector
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u2.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/dc_poe_injector
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/dc_poe_injector
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_capclear.kicad_pcb step3_diffpairs.kicad_pcb '*Eth*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1402 --diff-pair-gap 0.127 --clearance-ceiling 0.127 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.5 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/dc_poe_injector
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.5 --power-nets GND POE_45 VCC +3.3V /Power/VCC_BOOST --power-nets-widths 0.4 0.6 0.4 0.4 0.3 --max-ripup 5
