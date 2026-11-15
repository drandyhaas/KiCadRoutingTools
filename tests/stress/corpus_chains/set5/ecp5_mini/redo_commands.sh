#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/ecp5_mini.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.25 --power-nets +1V1 +5V +1V8 +2V5 --power-nets-widths 0.25 0.25 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb step2_fanout_u2.kicad_pcb --component U2 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+3V3' '!+1V1' '!+5V' '!+1V8' '!+2V5' --diff-pairs '/P*' '/HR_CK*' --diff-pair-gap 0.1 --clearance 0.1 --track-width 0.127 --via-size 0.4 --via-drill 0.2 --plane-net-layers GND:In1.Cu +3V3:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u2.kicad_pcb --component U2 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+3V3' '!+1V1' '!+5V' '!+1V8' '!+2V5' --diff-pairs '/P*' '/HR_CK*' --diff-pair-gap 0.1 --clearance 0.1 --track-width 0.127 --via-size 0.4 --via-drill 0.2 --plane-net-layers GND:In1.Cu +3V3:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u2b.kicad_pcb --component U2 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+3V3' '!+1V1' '!+5V' '!+1V8' '!+2V5' --diff-pairs '/P*' '/HR_CK*' --diff-pair-gap 0.1 --clearance 0.1 --track-width 0.1 --via-size 0.3 --via-drill 0.18 --plane-net-layers GND:In1.Cu +3V3:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u2b.kicad_pcb step2b_clearance_u2.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2b_clearance_u2.kicad_pcb --output step3_fanout_u3.kicad_pcb --component U3 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+3V3' '!+1V1' '!+5V' '!+1V8' '!+2V5' --diff-pairs '/HR_CK*' --diff-pair-gap 0.127 --clearance 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.25 --plane-net-layers GND:In1.Cu +3V3:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_fanout_u3.kicad_pcb step3b_clearance_u3.kicad_pcb --clearance 0.127
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3b_clearance_u3.kicad_pcb step4_diff.kicad_pcb '/P*' '/HR_CK*' '/USB_D*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb final_board.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +5V +1V8 +2V5 --power-nets-widths 0.127 0.127 0.25 0.25 0.25 0.25 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb final_board.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +5V +1V8 +2V5 --power-nets-widths 0.127 0.127 0.25 0.25 0.25 0.25 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb final_board.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +5V +1V8 +2V5 --power-nets-widths 0.127 0.127 0.25 0.25 0.25 0.25 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb final_board.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +5V +1V8 +2V5 --power-nets-widths 0.127 0.127 0.25 0.25 0.25 0.25 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb final_board.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +5V +1V8 +2V5 --power-nets-widths 0.127 0.127 0.25 0.25 0.25 0.25 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py final_board.kicad_pcb step6_retry.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +5V +1V8 +2V5 --power-nets-widths 0.127 0.127 0.25 0.25 0.25 0.25 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ecp5_mini
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_retry.kicad_pcb step7_fix.kicad_pcb /PE26+ /PE26- --rip-existing-nets /PE26+ /PE26- --force-reroute --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +5V +1V8 +2V5 --power-nets-widths 0.127 0.127 0.25 0.25 0.25 0.25
