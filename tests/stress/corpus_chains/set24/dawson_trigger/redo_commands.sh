#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set24/dawson_trigger.kicad_pcb --component U3 --output step1_fanout_u3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_fanout_u3.kicad_pcb --component U2 --output step2_fanout_u2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2_fanout_u2.kicad_pcb --component U4 --output step3_fanout_u4.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step3_fanout_u4.kicad_pcb --component U5 --output step4_fanout_u5.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step4_fanout_u5.kicad_pcb --component U6 --output step5_fanout_u6.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step5_fanout_u6.kicad_pcb --component U7 --output step6_fanout_u7.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py step6_fanout_u7.kicad_pcb step7_planes_gnd.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.6 --via-drill 0.3 --clearance-ceiling 0.2 --track-width 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/dawson_trigger
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step7_planes_gnd.kicad_pcb step8_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V +3.4V +1.7V +33.1V +34.3V --power-nets-widths 0.3 0.4 0.3 0.3 0.3 0.3
