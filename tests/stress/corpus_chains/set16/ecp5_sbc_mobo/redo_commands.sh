#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
# REBUILT from redo_commands.timings.jsonl after a relaunch clobbered the
# original (see run_board.sh archive-on-relaunch fix). Chain walked backward
# from the graded board (step5_gnd_fix.kicad_pcb); commands are byte-faithful re-quotes.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ecp5_sbc_mobo
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set16/ecp5_sbc_mobo.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.1524 --via-size 0.4572 --via-drill 0.254 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ecp5_sbc_mobo
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --output step2a_fanout_u1.kicad_pcb --clearance 0.1524 --width 0.1524 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ecp5_sbc_mobo
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u1.kicad_pcb --component U5 --output step2b_fanout_u5.kicad_pcb --clearance 0.1524 --width 0.1524 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ecp5_sbc_mobo
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_u5.kicad_pcb --component U3 --output step2c_fanout_u3.kicad_pcb --escape-method underpad --allow-via-in-pad --clearance 0.09 --width 0.1 --via-size 0.25 --via-drill 0.15 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ecp5_sbc_mobo
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fanout_u3.kicad_pcb --component XM1 --output step2d_fanout_xm1.kicad_pcb --escape-method underpad --allow-via-in-pad --clearance 0.09 --width 0.1 --via-size 0.25 --via-drill 0.15 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ecp5_sbc_mobo
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2d_fanout_xm1.kicad_pcb --component XM2 --output step2e_fanout_xm2.kicad_pcb --escape-method underpad --allow-via-in-pad --clearance 0.09 --width 0.1 --via-size 0.25 --via-drill 0.15 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ecp5_sbc_mobo
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2e_fanout_xm2.kicad_pcb step2f_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ecp5_sbc_mobo
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2f_capclear.kicad_pcb step3_diff.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ecp5_sbc_mobo
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.1524 --track-width 0.1 --via-size 0.4572 --via-drill 0.254 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V +1V8 /HDMI/DVI_DVDD /HDMI/DVI_PVDD /HDMI/DVI_TVDD '/USB-C Power & Debug Port/VBUS' '/USB-C Power & Debug Port/VBUS_IN' --power-nets-widths 0.3 0.3 0.3 0.25 0.25 0.25 0.25 0.4 0.4 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/ecp5_sbc_mobo
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_route.kicad_pcb step5_gnd_fix.kicad_pcb --nets GND --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.09 --track-width 0.0762 --via-size 0.4572 --via-drill 0.254 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND --power-nets-widths 0.3 --max-ripup 5 --no-bga-zone
