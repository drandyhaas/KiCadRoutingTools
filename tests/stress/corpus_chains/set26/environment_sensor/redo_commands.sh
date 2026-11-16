#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/environment_sensor.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 GNDREF --plane-layers In1.Cu In2.Cu In1.Cu --via-size 0.5 --via-drill 0.2 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' '!GNDREF' --clearance 0.09 --via-size 0.5 --via-drill 0.2 --grid-step 0.05 --output step2a_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2a_fanout_u1.kicad_pcb step2b_clearance_u1.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_clearance_u1.kicad_pcb --component U3 --nets '*' '!GND' '!+3V3' '!GNDREF' --clearance 0.09 --via-size 0.5 --via-drill 0.2 --grid-step 0.05 --output step2c_fanout_u3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2c_fanout_u3.kicad_pcb step2d_clearance_u3.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2d_clearance_u3.kicad_pcb step3_diff_usb.kicad_pcb --nets /USB_D_P /USB_D_N --layers F.Cu In1.Cu In2.Cu B.Cu --impedance 90 --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.2 --grid-step 0.05 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2d_clearance_u3.kicad_pcb step3b_diff_usb_retry.kicad_pcb --nets /USB_D_P /USB_D_N --layers F.Cu In1.Cu In2.Cu B.Cu --impedance 90 --track-width 0.09 --diff-pair-gap 0.09 --clearance-ceiling 0.09 --via-size 0.5 --via-drill 0.2 --grid-step 0.05 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3b_diff_usb_retry.kicad_pcb step4_route_all.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND +3V3 GNDREF +5V /VBUS 'Net-(U3-VDDA)' 'Net-(U3-VBAT)' --power-nets-widths 0.3 0.3 0.2 0.3 0.3 0.2 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/environment_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route_all.kicad_pcb step5_route_mp_pads.kicad_pcb --nets 'unconnected-(J2-PadMP)' 'unconnected-(J3-PadMP)' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05
