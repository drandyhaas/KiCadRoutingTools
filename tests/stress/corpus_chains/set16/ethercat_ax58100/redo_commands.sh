#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
# REBUILT from redo_commands.timings.jsonl after a relaunch clobbered the
# original (see run_board.sh archive-on-relaunch fix). Chain walked backward
# from the graded board (step5_route.kicad_pcb); commands are byte-faithful re-quotes.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ethercat_ax58100
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set16/ethercat_ax58100.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.15 --hole-to-hole-clearance 0.25 --via-size 0.5 --via-drill 0.3 --track-width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ethercat_ax58100
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2_fanout_u10.kicad_pcb --component U10 --nets '*' '!GND' --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ethercat_ax58100
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u10.kicad_pcb step2b_capclear_u10.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ethercat_ax58100
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_capclear_u10.kicad_pcb -o step3_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.13
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ethercat_ax58100
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_fanout_u6.kicad_pcb step3b_capclear_u6.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ethercat_ax58100
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3b_capclear_u6.kicad_pcb step4_diffpairs.kicad_pcb P0_RXIP P0_RXIN P0_TXOP P0_TXON P1_RXIP P1_RXIN P1_TXOP P1_TXON --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.25 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ethercat_ax58100
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb '*' --power-nets GND +3.3V +5V +1V2 +3.3VA '?24V_FIELD' --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.4 --track-width 0.13 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.25 --via-size 0.5 --via-drill 0.3 --grid-step 0.05 --max-ripup 5
