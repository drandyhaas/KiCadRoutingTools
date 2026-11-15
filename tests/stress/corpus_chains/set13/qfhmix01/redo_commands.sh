#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
# REBUILT from redo_commands.timings.jsonl after a relaunch clobbered the
# original (see run_board.sh archive-on-relaunch fix). Chain walked backward
# from the graded board (step9_retry.kicad_pcb); commands are byte-faithful re-quotes.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/qfhmix01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set13/qfhmix01.kicad_pcb --component U1 --output step1_fanout_U1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/qfhmix01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_U1.kicad_pcb --component U9 --output step2_fanout_U9.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/qfhmix01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_fanout_U9.kicad_pcb --component U11 --output step3_fanout_U11.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/qfhmix01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_fanout_U11.kicad_pcb --component U4 --escape-method underpad --via-size 0.35 --width 0.12 --clearance 0.1 --allow-via-in-pad --output step4_fanout_U4.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/qfhmix01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step4_fanout_U4.kicad_pcb step5_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/qfhmix01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step5_capclear.kicad_pcb step6_diff.kicad_pcb '*CLK_MIX1*' '*CLK_MIX2*' '*LNA_OUT_1*' '*LNA_OUT_2*' '*QFH_IN_1*' '*QFH_IN_2*' 'CLK_IN_*' 'DEM_OUTA_*' 'DEM_OUTB_*' --track-width 0.4 --diff-pair-gap 0.25 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/qfhmix01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step6_diff.kicad_pcb step7_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.45 --via-drill 0.3 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2
# recovered from agent_narrative.md:75 (recorded to no manifest)
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/qfhmix01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_planes.kicad_pcb step8_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5 --power-nets GND +5V VDDA +3V3 +3.3VA VCC --power-nets-widths 0.4 0.4 0.4 0.4 0.3 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/qfhmix01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step8_route.kicad_pcb step9_retry.kicad_pcb --nets /LNA/QFH_IN_1_P /LNA/QFH_IN_1_N /LNA/QFH_IN_2_P /LNA/QFH_IN_2_N --clearance-ceiling 0.1 --track-width 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5
