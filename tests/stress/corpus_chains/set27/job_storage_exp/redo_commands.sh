#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set27/job_storage_exp.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.3 --clearance-ceiling 0.15 --zone-clearance 0.15 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1_retry.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.09 --grid-step 0.05 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_u1_retry.kicad_pcb step3_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3_clearance.kicad_pcb step4a_diff_pcie.kicad_pcb 'PCIE2_*' --impedance 85 --diff-pair-gap 0.15 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4a_diff_pcie.kicad_pcb step4b_diff_usb.kicad_pcb 'USB1_D_*' '*USBDN_D2*' '*USBDN_D3*' --impedance 90 --diff-pair-gap 0.15 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4b_diff_usb.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 +5V '/USB and SD Card/USBC1_VBUS' '/USB and SD Card/USBA1_VBUS' 'Net-(U1-VDD18PLL)' 'Net-(U1-VDD18)' --power-nets-widths 0.3 0.3 0.4 0.4 0.4 0.25 0.25 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3_clearance.kicad_pcb step4a_diff_pcie_v2.kicad_pcb 'PCIE2_*' --impedance 85 --diff-pair-gap 0.15 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias --polarity-swap-nets 'PCIE2_TX0*'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/job_storage_exp
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_route.kicad_pcb step6_route_fix.kicad_pcb --nets 'unconnected-(D8-CH3-Pad4)' 'unconnected-(D8-CH4-Pad5)' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05
