#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set23/overlord_cm5.kicad_pcb step1_planes.kicad_pcb --nets GND GND --plane-layers In1.Cu In4.Cu --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout_module1.kicad_pcb --component Module1 --escape-method underpad --allow-via-in-pad --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.1 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_module1.kicad_pcb --component Module1 --escape-method underpad --allow-via-in-pad --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.1 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_module1.kicad_pcb step2b_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_clearance.kicad_pcb --output step3_fanout_u1.kicad_pcb --component U1 --clearance 0.1 --via-size 0.45 --via-drill 0.3 --width 0.15 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3_fanout_u1.kicad_pcb step4_diff_eth.kicad_pcb 'ETH_*' --clearance-ceiling 0.127 --diff-pair-gap 0.127 --track-width 0.1524 --impedance 100 --via-size 0.45 --via-drill 0.3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4_diff_eth.kicad_pcb step5_diff_cm5usb.kicad_pcb '/CM5/D*' --clearance-ceiling 0.1 --diff-pair-gap 0.1 --track-width 0.1524 --impedance 90 --via-size 0.45 --via-drill 0.3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_diff_cm5usb.kicad_pcb step6_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --track-width 0.16 --grid-step 0.05 --via-size 0.45 --via-drill 0.3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --power-nets GND +5V +3.3V --power-nets-widths 0.3 0.4 0.3 --max-ripup 5 --no-bga-zones
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/overlord_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_diff_cm5usb.kicad_pcb step6_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --track-width 0.16 --grid-step 0.05 --via-size 0.45 --via-drill 0.3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --power-nets GND +5V +3.3V --power-nets-widths 0.3 0.4 0.3 --max-ripup 5 --no-bga-zones
