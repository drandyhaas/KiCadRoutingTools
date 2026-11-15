#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
# REBUILT from redo_commands.timings.jsonl after a relaunch clobbered the
# original (see run_board.sh archive-on-relaunch fix). Chain walked backward
# from the graded board (step5_route.kicad_pcb); commands are byte-faithful re-quotes.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/argus_mainboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set16/argus_mainboard.kicad_pcb step1_planes.kicad_pcb --nets GND GND +3V3_M +3V3_P --plane-layers In1.Cu In2.Cu In2.Cu In2.Cu --clearance-ceiling 0.1397 --via-size 0.6096 --via-drill 0.3048 --track-width 0.127 --hole-to-hole-clearance 0.2 --board-edge-clearance 0.381
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/argus_mainboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3_M' '!+3V3_P' --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/argus_mainboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step3_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/argus_mainboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_clearance.kicad_pcb --output step4_diffpairs.kicad_pcb --nets /MCU_IO/D+ /MCU_IO/D- /USB/D_UP+ /USB/D_UP- /USB/USB_D+ /USB/USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.6096 --via-drill 0.3048 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias --hole-to-hole-clearance 0.2 --board-edge-clearance 0.381
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/argus_mainboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb --output step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.1397 --track-width 0.127 --via-size 0.6096 --via-drill 0.3048 --hole-to-hole-clearance 0.2 --board-edge-clearance 0.381 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0 --grid-step 0.05 --no-bga-zone --max-ripup 5 --power-nets GND +3V3_M +3V3_P VBATT SYSTEM_VSS SYSTEM_VDD VCHRG +3V3_IMU +1V1 +3V3_MS +3V3_PS +3V3_RTC VDD_LoRa VDD_LS VDD_GPS +3V3_SD 'Net-(U1-ADC_AVDD)' 'Net-(U1-VREG_AVDD)' --power-nets-widths 0.3 0.3 0.3 0.4 0.25 0.25 0.3 0.25 0.25 0.25 0.25 0.25 0.25 0.25 0.25 0.25 0.2 0.2
