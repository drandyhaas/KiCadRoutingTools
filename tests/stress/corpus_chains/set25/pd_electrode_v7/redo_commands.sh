#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/pd_electrode_v7
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/pd_electrode_v7
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set25/pd_electrode_v7.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set25/pd_electrode_v7/step1_plane.kicad_pcb --nets /POUR --plane-layers F.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.105 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/pd_electrode_v7
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/pd_electrode_v7
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set25/pd_electrode_v7/step1_plane.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set25/pd_electrode_v7/step2_route.kicad_pcb '*' --clearance-ceiling 0.105 --track-width 0.105 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets /POUR --power-nets-widths 0.3 --stats
