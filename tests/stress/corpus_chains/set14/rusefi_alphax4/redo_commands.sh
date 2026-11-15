#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/rusefi_alphax4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/rusefi_alphax4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/rusefi_alphax4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/rusefi_alphax4.kicad_pcb board_step1_planes.kicad_pcb --nets GND GNDA +12V_RAW --plane-layers In1.Cu In1.Cu In2.Cu --clearance-ceiling 0.197 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.29999
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/rusefi_alphax4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py board_step1_planes.kicad_pcb board_step2a_diff.kicad_pcb --nets /CAN+ /CAN- /IN_CAM_VR+ /IN_CAM_VR- /IN_CRANK+ /IN_CRANK- /MCU/USB+ /MCU/USB- /OUT_ETB+ /OUT_ETB- 'Net-(U1-IN+)' 'Net-(U1-IN-)' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2032 --diff-pair-gap 0.2 --clearance-ceiling 0.197 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.29999 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/rusefi_alphax4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py board_step1_planes.kicad_pcb board_step2a_diff.kicad_pcb --nets /CAN+ /CAN- /IN_CAM_VR+ /IN_CAM_VR- /IN_CRANK+ /IN_CRANK- /MCU/USB+ /MCU/USB- /OUT_ETB+ /OUT_ETB- 'Net-(U1-IN+)' 'Net-(U1-IN-)' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2032 --diff-pair-gap 0.2 --clearance-ceiling 0.197 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.29999 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/rusefi_alphax4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/rusefi_alphax4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board_step2a_diff.kicad_pcb board_final.kicad_pcb --nets '*' --no-bga-zone P1 P2 --max-ripup 5 --power-nets GND GNDA +12V_RAW +12V +12V_ETB +5VA +3.3VA +3.3V /HALL_OPT_PU/+12V_LIM +5VAS /MCU/VBUS +5V --power-nets-widths 0.4 0.3 0.6 0.5 0.5 0.3 0.3 0.3 0.3 0.3 0.3 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.197 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.29999 --track-width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/rusefi_alphax4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board_final.kicad_pcb board_final_retry.kicad_pcb --nets GND /OUT_VVT1 /OUT_INJ4 --grid-step 0.025 --via-size 0.25 --via-drill 0.15 --clearance-ceiling 0.09 --track-width 0.0889 --hole-to-hole-clearance 0.2 --max-ripup 8 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --power-nets GND --power-nets-widths 0.4
