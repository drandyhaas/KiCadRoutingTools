#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/odmr_adf_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/odmr_adf_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/odmr_adf_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/odmr_adf_board.kicad_pcb step1_planes.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.15 --zone-clearance 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/odmr_adf_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout.kicad_pcb --component IC401 --nets '*' '!GND' '!+3.3V' --clearance 0.15 --width 0.15 --grid-step 0.05 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/odmr_adf_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component IC401 --nets '*' '!GND' '!+3.3V' --clearance 0.15 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/odmr_adf_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step2b_clearance.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/odmr_adf_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/odmr_adf_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_clearance.kicad_pcb --output step3_diff.kicad_pcb --nets 'Net-(IC401-RFOUTA+)' 'Net-(IC401-RFOUTA-)' /MW_Source/RFB+ /MW_Source/RFB- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/odmr_adf_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/odmr_adf_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb --output step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3.3V +5V VDD 'Net-(Y401-VCC)' 'Net-(D307-VDD)' +3.3Vx --power-nets-widths 0.3 0.3 0.3 0.2 0.2 0.2 0.2 --max-ripup 5 --no-bga-zone
