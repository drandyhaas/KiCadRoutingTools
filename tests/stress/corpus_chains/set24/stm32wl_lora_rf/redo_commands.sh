#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set24/stm32wl_lora_rf.kicad_pcb --component U1 --output step1_qfn_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py step1_qfn_fanout.kicad_pcb step2_planes.kicad_pcb --nets GND VDD --plane-layers In1.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.6 --via-drill 0.3 --track-width 0.2 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --power-nets GND VDD --power-nets-widths 0.2 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2_planes.kicad_pcb step3_diff.kicad_pcb --nets /RFI_P /RFI_N --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --via-size 0.6 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_rf50ohm.kicad_pcb --nets /VR_PA /RFO_HP /RFO_IN /ANT_CON /RFSWT_RFIN /RF_MATCH /IPD_RFIN /RF2 /RF_OUT --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.156464 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_rf50ohm.kicad_pcb step5_fixwidth.kicad_pcb --nets /DRDY /TX /RX VDDA /FE_CTRL /OSCIN 'Net-(U1-PH3-BOOT0)' /OSCOUT VDRF1V55 'Net-(U1-VLXSMPS)' /NRST /SWDIO /SWCLK --force-reroute --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_fixwidth.kicad_pcb step6_vddbat.kicad_pcb --nets VDD_BAT --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.5 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/stm32wl_lora_rf
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_vddbat.kicad_pcb step7_final.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VDD VDDA VDD_DEBUGER 'Net-(U3-VDD)' --power-nets-widths 0.3 0.3 0.25 0.25 0.25 --max-ripup 5 --no-bga-zone
