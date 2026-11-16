#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set20/epr_bridge_ctrl.kicad_pcb -c U1 --width 0.2 --clearance 0.09 -o step1_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_u1.kicad_pcb -c U2 --width 0.2 --clearance 0.09 -o step2_fanout_u2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set20/epr_bridge_ctrl.kicad_pcb step1_planes.kicad_pcb --nets GND +5V --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.15 --zone-clearance 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V VBUS +2V5 +3V3 --power-nets-widths 0.3 0.4 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -c U1 --width 0.2 --clearance 0.09 --nets '!GND' '!+5V' -o step2_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_fanout_u1.kicad_pcb -c U2 --width 0.2 --clearance 0.09 --nets '!GND' '!+5V' -o step3_fanout_u2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_fanout_u2.kicad_pcb step4_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4_capclear.kicad_pcb step5_diffpair.kicad_pcb 'Net-(U2-USBD+)' 'Net-(U2-USBD-)' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/epr_bridge_ctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diffpair.kicad_pcb step6_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.15 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V VBUS +2V5 +3V3 --power-nets-widths 0.3 0.4 0.4 0.3 0.3 --max-ripup 5
