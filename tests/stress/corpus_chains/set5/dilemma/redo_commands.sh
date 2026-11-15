#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/dilemma.kicad_pcb step1_planes.kicad_pcb --nets GND GND_L +3V3 +3V3_L --plane-layers In1.Cu In1.Cu In2.Cu In2.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.15 --grid-step 0.1 --output step2a_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u1.kicad_pcb --component U5002 --nets '*' '!GND_L' '!+3V3_L' --clearance 0.09 --width 0.15 --grid-step 0.1 --output step2b_fanout_u5002.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u5002.kicad_pcb step3_capopt.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capopt.kicad_pcb step4_diffpairs.kicad_pcb '*' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb '*' --clearance-ceiling 0.127 --track-width 0.16 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND GND_L +3V3 +3V3_L VBUS VBUS_L +1V1 +1V1_L +5V +5V_L /VBUS_DET /VBUS_DET_L --power-nets-widths 0.3 0.3 0.3 0.3 0.4 0.4 0.25 0.25 0.4 0.4 0.2 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/dilemma
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step5b_retry.kicad_pcb '*' --clearance-ceiling 0.127 --track-width 0.16 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND GND_L +3V3 +3V3_L VBUS VBUS_L +1V1 +1V1_L +5V +5V_L /VBUS_DET /VBUS_DET_L --power-nets-widths 0.3 0.3 0.3 0.3 0.4 0.4 0.25 0.25 0.4 0.4 0.2 0.2
