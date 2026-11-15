#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
# REBUILT from redo_commands.timings.jsonl after a relaunch clobbered the
# original (see run_board.sh archive-on-relaunch fix). Chain walked backward
# from the graded board (step5_route.kicad_pcb); commands are byte-faithful re-quotes.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/z80_devboard_rp2040
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/z80_devboard_rp2040.kicad_pcb step1_planes.kicad_pcb --nets GND Vcc --plane-layers In1.Cu In2.Cu --via-size 0.5 --via-drill 0.3 --track-width 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/z80_devboard_rp2040
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component RP2040 --nets '*' '!GND' '!Vcc' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --clearance 0.09 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/z80_devboard_rp2040
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step3_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/z80_devboard_rp2040
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpairs.kicad_pcb '*USB_D*' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/z80_devboard_rp2040
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4_diffpairs.kicad_pcb step4b_diffpairs2.kicad_pcb USB-RP2040-DP USB-RP2040-DM --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/z80_devboard_rp2040
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4b_diffpairs2.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND Vcc RP2040-ADC_AVDD RP2040-DVDD USB-C-VBUS --power-nets-widths 0.4 0.4 0.3 0.3 0.4
