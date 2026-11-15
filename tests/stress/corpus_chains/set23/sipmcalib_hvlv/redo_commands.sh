#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/sipmcalib_hvlv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set23/sipmcalib_hvlv.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set23/sipmcalib_hvlv/step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --zone-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/sipmcalib_hvlv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set23/sipmcalib_hvlv/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set23/sipmcalib_hvlv/step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +5V --power-nets-widths 0.4 0.4
