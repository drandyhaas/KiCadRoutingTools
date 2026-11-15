#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lpddr4_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lpddr4_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lpddr4_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lpddr4_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set1/lpddr4_testbed.kicad_pcb step1_planes.kicad_pcb --nets GND VDD2 GND --plane-layers In1.Cu In3.Cu In4.Cu --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lpddr4_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' '!VDD2' --diff-pairs 'CK_*' 'DQ_S*' --diff-pair-gap 0.1 --clearance 0.1 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --plane-net-layers GND:In1.Cu,In4.Cu VDD2:In3.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lpddr4_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2b_fanout.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' '!VDD2' --diff-pairs 'CK_*' 'DQ_S*' --diff-pair-gap 0.1 --clearance 0.1 --track-width 0.08 --via-size 0.25 --via-drill 0.15 --plane-net-layers GND:In1.Cu,In4.Cu VDD2:In3.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lpddr4_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lpddr4_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpairs.kicad_pcb 'CK_*' 'DQ_S*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.25 --via-drill 0.15 --no-gnd-vias --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lpddr4_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lpddr4_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND VDD2 VDDQ VDD1 VDDLED --power-nets-widths 0.3 0.3 0.3 0.25 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lpddr4_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step6_retry.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.0762 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND VDD2 VDDQ VDD1 VDDLED --power-nets-widths 0.3 0.3 0.3 0.25 0.2
