#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tk44
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set17/tk44.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.45 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tk44
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb /DATA+ /DATA- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/tk44
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --power-nets GND /VBUS /VBAT /nRF_VDD --power-nets-widths 0.4 0.4 0.4 0.4 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
