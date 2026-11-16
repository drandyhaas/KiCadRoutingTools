#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/reaction_wheel_foc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/reaction_wheel_foc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/reaction_wheel_foc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/reaction_wheel_foc.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/reaction_wheel_foc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_u2.kicad_pcb --component U2 --clearance 0.2 --width 0.2 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/reaction_wheel_foc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u2.kicad_pcb --component U2 --clearance 0.2 --width 0.2 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/reaction_wheel_foc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_fanout_u2.kicad_pcb --output step2b_fanout_u4.kicad_pcb --component U4 --clearance 0.1 --width 0.15 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/reaction_wheel_foc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/reaction_wheel_foc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2b_fanout_u4.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/reaction_wheel_foc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/reaction_wheel_foc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2c_capclear.kicad_pcb step3_diffpairs.kicad_pcb --nets /USB_D+ /USB_D- /USBR_D+ /USBR_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/reaction_wheel_foc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3.3V +5V VBAT +3.3VA 'Net-(U4-AVDD)' --power-nets-widths 0.3 0.3 0.3 0.4 0.25 0.2
