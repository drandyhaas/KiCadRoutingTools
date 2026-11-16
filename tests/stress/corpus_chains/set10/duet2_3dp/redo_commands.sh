#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/duet2_3dp.kicad_pcb step1_planes.kicad_pcb --nets GND V_FUSED --plane-layers In2.Cu In1.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.45 --via-drill 0.35052 --track-width 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2_fanout_U1.kicad_pcb -c U1 --nets '*' '!GND' --width 0.15 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_U1.kicad_pcb step2b_capclear_U1.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_capclear_U1.kicad_pcb -o step3_fanout_U5.kicad_pcb -c U5 --nets '*' '!GND' '!V_FUSED' --width 0.2 --clearance 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_capclear_U1.kicad_pcb -o step3_fanout_U5.kicad_pcb -c U5 --nets '*' '!GND' '!V_FUSED' --width 0.15 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_fanout_U5.kicad_pcb step3b_capclear_U5.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3b_capclear_U5.kicad_pcb -o step4_fanout_U6.kicad_pcb -c U6 --nets '*' '!GND' '!V_FUSED' --width 0.15 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step4_fanout_U6.kicad_pcb step4b_capclear_U6.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step4b_capclear_U6.kicad_pcb -o step5_fanout_U7.kicad_pcb -c U7 --nets '*' '!GND' '!V_FUSED' --width 0.15 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step5_fanout_U7.kicad_pcb step5b_capclear_U7.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5b_capclear_U7.kicad_pcb -o step6_fanout_U8.kicad_pcb -c U8 --nets '*' '!GND' '!V_FUSED' --width 0.15 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step6_fanout_U8.kicad_pcb step6b_capclear_U8.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step6b_capclear_U8.kicad_pcb -o step7_fanout_U9.kicad_pcb -c U9 --nets '*' '!GND' '!V_FUSED' --width 0.15 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step7_fanout_U9.kicad_pcb step7b_capclear_U9.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7b_capclear_U9.kicad_pcb step8_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.15 --via-size 0.45 --via-drill 0.35052 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets V_IN BED- E0- E1- '/Heater & Fan Control/vfanfused' V_FAN X_MOT_1A X_MOT_1B X_MOT_2A X_MOT_2B Y_MOT_1A Y_MOT_1B Y_MOT_2A Y_MOT_2B Z_MOT_1A Z_MOT_1B Z_MOT_2A Z_MOT_2B E0_MOT_1A E0_MOT_1B E0_MOT_2A E0_MOT_2B E1_MOT_1A E1_MOT_1B E1_MOT_2A E1_MOT_2B +3.3V 5V_INT 5V_EXT 5V_EXT_INPUT FAN0- FAN1- FAN2- '/Power Supplies/vin' /Processor/VDDCORE VBUS ESP_3.3VIN FAN0 FAN1 FAN2 GND V_FUSED --power-nets-widths 0.5 0.5 0.5 0.5 0.5 0.5 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/duet2_3dp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7b_capclear_U9.kicad_pcb step8_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.15 --via-size 0.45 --via-drill 0.35052 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets V_IN BED- E0- E1- '/Heater & Fan Control/vfanfused' V_FAN X_MOT_1A X_MOT_1B X_MOT_2A X_MOT_2B Y_MOT_1A Y_MOT_1B Y_MOT_2A Y_MOT_2B Z_MOT_1A Z_MOT_1B Z_MOT_2A Z_MOT_2B E0_MOT_1A E0_MOT_1B E0_MOT_2A E0_MOT_2B E1_MOT_1A E1_MOT_1B E1_MOT_2A E1_MOT_2B +3.3V 5V_INT 5V_EXT 5V_EXT_INPUT FAN0- FAN1- FAN2- '/Power Supplies/vin' /Processor/VDDCORE VBUS ESP_3.3VIN FAN0 FAN1 FAN2 GND V_FUSED --power-nets-widths 0.5 0.5 0.5 0.5 0.5 0.5 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3
