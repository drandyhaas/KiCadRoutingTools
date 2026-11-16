#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/dlr_sensor_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/dlr_sensor_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/dlr_sensor_hat.kicad_pcb -c U2 --output step1_qfn_fanout.kicad_pcb --width 0.2 --clearance 0.15 --diff-pairs /USB_DP,/USB_DM
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/dlr_sensor_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/dlr_sensor_hat.kicad_pcb -c U2 --output step1_qfn_fanout.kicad_pcb --width 0.2 --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/dlr_sensor_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/dlr_sensor_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/dlr_sensor_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_qfn_fanout.kicad_pcb --output step2_diff.kicad_pcb --nets /USB_DP /USB_DM --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/dlr_sensor_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/dlr_sensor_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step2_diff.kicad_pcb step3_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --track-width 0.2 --power-nets GND --power-nets-widths 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/dlr_sensor_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/dlr_sensor_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_planes.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V --power-nets-widths 0.3 0.3 0.4 --max-ripup 5 --no-bga-zone
