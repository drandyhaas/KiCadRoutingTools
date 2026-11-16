#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/linht_radio.kicad_pcb step1_planes.kicad_pcb --nets GND GND +3V3 --plane-layers In1.Cu In4.Cu In2.Cu --clearance-ceiling 0.11 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fan_U1.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.11 --width 0.2 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fan_U1.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.11 --width 0.2 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fan_U1.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.2 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fan_U1.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.11 --width 0.2 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fan_U1.kicad_pcb --output step2b_fan_IC1.kicad_pcb --component IC1 --nets '*' '!GND' '!+3V3' --clearance 0.11 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fan_IC1.kicad_pcb --output step2c_fan_U10.kicad_pcb --component U10 --nets '*' '!GND' '!+3V3' --clearance 0.11 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fan_U10.kicad_pcb --output step2d_fan_U11.kicad_pcb --component U11 --nets '*' '!GND' '!+3V3' --clearance 0.11 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2d_fan_U11.kicad_pcb --output step2e_fan_U2.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' --clearance 0.11 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2e_fan_U2.kicad_pcb --output step2f_fan_U7.kicad_pcb --component U7 --nets '*' '!GND' '!+3V3' --clearance 0.11 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2f_fan_U7.kicad_pcb --output step2g_fan_U9.kicad_pcb --component U9 --nets '*' '!GND' '!+3V3' --clearance 0.11 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2g_fan_U9.kicad_pcb --output step2h_fan_U12.kicad_pcb --component U12 --nets '*' '!GND' '!+3V3' --clearance 0.1 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2h_fan_U12.kicad_pcb --output step2i_fan_U14.kicad_pcb --component U14 --nets '*' '!GND' '!+3V3' --clearance 0.1 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2i_fan_U14.kicad_pcb step3_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_clearance.kicad_pcb --output step4a_diff_usb.kicad_pcb --nets /Connectors/USB_DP /Connectors/USB_DN /Connectors/xUDP /Connectors/xUDN --impedance 90 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4a_diff_usb.kicad_pcb --output step4b_diff_u12.kicad_pcb --nets 'Net-(U12-D+)' 'Net-(U12-D-)' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4b_diff_u12.kicad_pcb --output step5_rf.kicad_pcb --nets /RF/ATT_IN_OUT /RF/RF_IN /RF/RF_IN_ATT /RF/RF_OUT 'Net-(C52-Pad2)' 'Net-(C54-Pad2)' 'Net-(C55-Pad1)' 'Net-(C56-Pad2)' 'Net-(FL1-IN)' 'Net-(FL1-OUT)' 'Net-(IC1-RFI)' 'Net-(IC1-RFO_P)' 'Net-(U2-RF_IN)' 'Net-(U2-RF_OUT_1)' 'Net-(U2-RF_OUT_2)' 'Net-(U9-RF1)' 'Net-(U9-RF2)' 'Net-(U9-RF3)' --impedance 50 --clearance-ceiling 0.17 --layers F.Cu --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_rf.kicad_pcb --output step6_route.kicad_pcb --nets '*' '!Net-(C52-Pad2)' '!Net-(C56-Pad2)' '!Net-(U9-RF3)' '!Net-(U9-RF2)' '!Net-(C54-Pad2)' '!Net-(U9-RF1)' '!/RF/RF_OUT' '!Net-(IC1-RFI)' '!Net-(FL1-IN)' '!/RF/ATT_IN_OUT' '!Net-(C55-Pad1)' '!Net-(FL1-OUT)' '!/RF/RF_IN' '!/RF/RF_IN_ATT' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.11 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --grid-step 0.1 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V +3.3VP +1V8 VBUS VDD --power-nets-widths 0.3 0.3 0.4 0.25 0.25 0.3 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/linht_radio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_route.kicad_pcb --output step7_retry.kicad_pcb --nets 'Net-(U12-TS)' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.09 --track-width 0.0889 --via-size 0.3 --via-drill 0.15 --grid-step 0.05 --hole-to-hole-clearance 0.2 --max-ripup 5
