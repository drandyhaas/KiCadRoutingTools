#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
# REBUILT from redo_commands.timings.jsonl after a relaunch clobbered the
# original (see run_board.sh archive-on-relaunch fix). Chain walked backward
# from the graded board (step4_route.kicad_pcb); commands are byte-faithful re-quotes.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set15/tildagon_base.kicad_pcb step1_planes.kicad_pcb --nets GND 3V3_SYS --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.15 --zone-clearance 0.15 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!3V3_SYS' --diff-pairs /USB+ /USB- --diff-pair-gap 0.1 --clearance 0.09 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --plane-net-layers GND:In1.Cu 3V3_SYS:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2a_fanout_u1.kicad_pcb step2a_fanout_u1_clr.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u1_clr.kicad_pcb --output step2b_fanout_u3.kicad_pcb --component U3 --nets '*' '!GND' '!3V3_SYS' --width 0.15 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_u3.kicad_pcb --output step2c_fanout_u24.kicad_pcb --component U24 --nets '*' '!GND' '!3V3_SYS' --width 0.15 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fanout_u24.kicad_pcb --output step2d_fanout_u25.kicad_pcb --component U25 --nets '*' '!GND' '!3V3_SYS' --width 0.15 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2d_fanout_u25.kicad_pcb --output step2e_fanout_u26.kicad_pcb --component U26 --nets '*' '!GND' '!3V3_SYS' --width 0.15 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2e_fanout_u26.kicad_pcb --output step2f_fanout_u2.kicad_pcb --component U2 --nets '*' '!GND' '!3V3_SYS' --width 0.15 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2f_fanout_u2.kicad_pcb --output step2g_fanout_u5.kicad_pcb --component U5 --nets '*' '!GND' '!3V3_SYS' --width 0.15 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2g_fanout_u5.kicad_pcb step2h_fanout_clr.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2h_fanout_clr.kicad_pcb step3_diff.kicad_pcb /DEV_USB+ /DEV_USB- /USB+ /USB- /USB/HOST_D+ /USB/HOST_D- --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/tildagon_base
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND 3V3_SYS /VBUS /VBUS_SW /USB/PMID /VSYS /bot-top-link/TOP_VBAT_SW --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.3 --max-ripup 5
