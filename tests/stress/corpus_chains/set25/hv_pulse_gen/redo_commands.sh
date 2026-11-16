#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/hv_pulse_gen
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/hv_pulse_gen
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set25/hv_pulse_gen.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set25/hv_pulse_gen/step1_planes.kicad_pcb --nets GND1 --plane-layers B.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/hv_pulse_gen
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/hv_pulse_gen
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set25/hv_pulse_gen/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set25/hv_pulse_gen/step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND1 GND PWRGND2 +12V +24CH1 3.3V HV HV2 HVCH1 VP1CHX VP3CHX VSCH1 --power-nets-widths 1.0 0.5 0.5 0.6 1.0 0.4 1.0 1.0 1.0 1.0 1.0 1.0
