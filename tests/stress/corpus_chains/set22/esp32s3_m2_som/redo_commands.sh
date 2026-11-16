#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/esp32s3_m2_som.kicad_pcb --component U3 --output step1_qfn_dryrun.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/esp32s3_m2_som.kicad_pcb step1_planes.kicad_pcb --nets Earth +3V3 --plane-layers In1.Cu In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_qfn.kicad_pcb --component U3 --nets '*' '!Earth' '!+3V3' --clearance 0.09 --track-width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_qfn.kicad_pcb --component U3 --nets '*' '!Earth' '!+3V3' --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_qfn.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2b_capclear.kicad_pcb step3_diff.kicad_pcb /USB_D+ /USB_D- /M.2_USB_D+ /M.2_USB_D- /SPICLK_P /SPICLK_N --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.45 --via-drill 0.3 --grid-step 0.05 --hole-to-hole-clearance 0.2 --power-nets Earth +3V3 VBUS VDD_SPI +3V3_RF --power-nets-widths 0.25 0.25 0.25 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.45 --via-drill 0.3 --grid-step 0.05 --hole-to-hole-clearance 0.2 --power-nets Earth +3V3 VBUS VDD_SPI +3V3_RF --power-nets-widths 0.25 0.25 0.25 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.45 --via-drill 0.3 --grid-step 0.05 --hole-to-hole-clearance 0.2 --power-nets Earth +3V3 VBUS VDD_SPI +3V3_RF --power-nets-widths 0.25 0.25 0.25 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.45 --via-drill 0.3 --grid-step 0.05 --hole-to-hole-clearance 0.2 --power-nets Earth +3V3 VBUS VDD_SPI +3V3_RF --power-nets-widths 0.25 0.25 0.25 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.45 --via-drill 0.3 --grid-step 0.05 --hole-to-hole-clearance 0.2 --power-nets Earth +3V3 VBUS VDD_SPI +3V3_RF --power-nets-widths 0.25 0.25 0.25 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.45 --via-drill 0.3 --grid-step 0.05 --hole-to-hole-clearance 0.2 --power-nets Earth +3V3 VBUS VDD_SPI +3V3_RF --power-nets-widths 0.25 0.25 0.25 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step4b_retry.kicad_pcb --nets /GPIO7 /GPIO8 Earth --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.0889 --via-size 0.3 --via-drill 0.15 --grid-step 0.025 --hole-to-hole-clearance 0.2 --rip-existing-nets '*' --max-ripup 5 --power-nets Earth +3V3 VBUS VDD_SPI +3V3_RF --power-nets-widths 0.25 0.25 0.25 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4_route.kicad_pcb step5_fix_spiclk.kicad_pcb /SPICLK_P /SPICLK_N --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/esp32s3_m2_som
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_fix_spiclk.kicad_pcb step5b_fix_spiclk.kicad_pcb --nets /SPICLK_N --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --via-size 0.3 --via-drill 0.15 --grid-step 0.025 --hole-to-hole-clearance 0.2 --rip-existing-nets '*' --max-ripup 5 --power-nets Earth +3V3 VBUS VDD_SPI +3V3_RF --power-nets-widths 0.25 0.25 0.25 0.2 0.15
