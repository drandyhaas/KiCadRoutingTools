#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/tmc2209_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/tmc2209_dev.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/tmc2209_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/tmc2209_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.1 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/tmc2209_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.1 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/tmc2209_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/tmc2209_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/tmc2209_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/tmc2209_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_capclear.kicad_pcb step3_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND Vdrive +5V +28V 'Net-(A1-VIN)' --power-nets-widths 0.4 0.6 0.3 0.4 0.4 --max-ripup 5
