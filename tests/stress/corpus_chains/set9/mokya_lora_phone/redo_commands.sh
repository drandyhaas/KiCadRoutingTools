#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/mokya_lora_phone.kicad_pcb step1_planes.kicad_pcb --nets GND VCC_1V8 --plane-layers In4.Cu In1.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_U2.kicad_pcb --component U2 --nets '*' '!GND' --width 0.1 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_U2.kicad_pcb --component U2 --nets '*' '!GND' --width 0.1 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2a_fanout_U2.kicad_pcb step2a_capclear_U2.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_capclear_U2.kicad_pcb --output step2b_fanout_U6.kicad_pcb --component U6 --nets '*' '!GND' --width 0.1199 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_U6.kicad_pcb --output step2c_fanout_U7.kicad_pcb --component U7 --nets '*' '!GND' --width 0.1199 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fanout_U7.kicad_pcb --output step2d_fanout_U3.kicad_pcb --component U3 --nets '*' '!GND' --width 0.1199 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2d_fanout_U3.kicad_pcb --output step2e_fanout_U5.kicad_pcb --component U5 --nets '*' '!GND' --width 0.1199 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2e_fanout_U5.kicad_pcb --output step2f_fanout_U12.kicad_pcb --component U12 --nets '*' '!GND' '!/Power/*' --width 0.1199 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2e_fanout_U5.kicad_pcb --output step2f_fanout_U12.kicad_pcb --component U12 --nets '*' '!GND' --width 0.1199 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2f_fanout_U12.kicad_pcb step2g_capclear_all.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2g_capclear_all.kicad_pcb step3a_diff_usb.kicad_pcb /MCU_RP2350B/USB_DP /MCU_RP2350B/USB_DN /MCU_RP2350B/USB_P /MCU_RP2350B/USB_N /USB_CON_P /USB_CON_N /USB_FLT_P /USB_FLT_N --impedance 90 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3a_diff_usb.kicad_pcb step3b_diff_rfi.kicad_pcb 'Net-(U7-RFI_P)' 'Net-(U7-RFI_N)' --impedance 100 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3b_diff_rfi.kicad_pcb step3c_diff_audio.kicad_pcb 'Net-(U6-OUTP)' 'Net-(U6-OUTN)' --track-width 0.1199 --diff-pair-gap 0.15 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3c_diff_audio.kicad_pcb step4_rf50.kicad_pcb --nets 'Net-(ANT1-FEED)' 'Net-(ANT1-GND)' 'Net-(ANT2-FEED)' 'Net-(C45-Pad1)' 'Net-(C46-Pad2)' 'Net-(C50-Pad2)' 'Net-(C53-Pad2)' 'Net-(C54-Pad1)' 'Net-(C63-Pad1)' 'Net-(C64-Pad1)' 'Net-(FL1-INPUT)' 'Net-(FL1-OUTPUT)' 'Net-(J5-In)' 'Net-(J6-In)' 'Net-(U10-RF_IN)' 'Net-(U11-AI)' 'Net-(U7-RFO)' 'Net-(U7-VR_PA)' 'Net-(U8-RF1)' 'Net-(U8-RF2)' 'Net-(U8-RFC)' --impedance 50 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --board-edge-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_rf50.kicad_pcb step5_signal.kicad_pcb --nets '*' '!Net-(ANT1-FEED)' '!Net-(ANT1-GND)' '!Net-(ANT2-FEED)' '!Net-(C45-Pad1)' '!Net-(C46-Pad2)' '!Net-(C50-Pad2)' '!Net-(C53-Pad2)' '!Net-(C54-Pad1)' '!Net-(C63-Pad1)' '!Net-(C64-Pad1)' '!Net-(FL1-INPUT)' '!Net-(FL1-OUTPUT)' '!Net-(J5-In)' '!Net-(J6-In)' '!Net-(U10-RF_IN)' '!Net-(U11-AI)' '!Net-(U7-RFO)' '!Net-(U7-VR_PA)' '!Net-(U8-RF1)' '!Net-(U8-RF2)' '!Net-(U8-RFC)' '!Net-(U7-RFI_P)' '!Net-(U7-RFI_N)' --power-nets GND VCC_1V8 VCC_3V3 VCC_1V1 VSYS /Power/VBAT+ /Power/VBAT_CHRG 'Net-(X2-VDD)' 'Net-(U7-VBAT)' 'Net-(U7-VDD_IN)' 'Net-(U10-VCC_RF)' /MCU_RP2350B/USB_VBUS_DET 'Net-(U2A-VREG_AVDD)' 'Net-(U8-~{CTRL{slash}VDD})' 'Net-(U12-VDD)' VUSB --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --track-width 0.1199 --via-size 0.45 --via-drill 0.2 --board-edge-clearance 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/mokya_lora_phone
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_signal.kicad_pcb step6_retry.kicad_pcb --nets 'Net-(U2E-GPIO43_ADC3)' GND 'Net-(U14-TS_BIAS)' --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --track-width 0.0889 --grid-step 0.05 --power-nets GND --power-nets-widths 0.3 --via-size 0.45 --via-drill 0.2 --board-edge-clearance 0.2 --max-ripup 5
