#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/laser_backplane_dvi
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set25/laser_backplane_dvi.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/laser_backplane_dvi
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/laser_backplane_dvi
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +12V -5V LD_short- --power-nets-widths 0.5 0.5 0.5 0.8 --no-bga-zone --max-ripup 5
