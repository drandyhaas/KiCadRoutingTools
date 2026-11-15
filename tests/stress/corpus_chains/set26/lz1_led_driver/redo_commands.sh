#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/lz1_led_driver.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver/step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver/step2_fanout_u1.kicad_pcb --component U1 --clearance 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver/step1_planes.kicad_pcb --output /Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver/step2_fanout_u1.kicad_pcb --component U1 --clearance 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py /Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver/step2_fanout_u1.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver/step3_capclear.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py /Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver/step3_capclear.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver/step4_diffpair.kicad_pcb --nets 'Net-(J2-D_{A}+)' 'Net-(J2-D_{A}-)' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver/step4_diffpair.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set26/lz1_led_driver/step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --grid-step 0.05 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND '/LED DRIVER & USB-C Power output/VBUS' +12V --power-nets-widths 0.25 0.25 0.25
