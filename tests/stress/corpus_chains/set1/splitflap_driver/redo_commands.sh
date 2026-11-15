#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/splitflap_driver
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/splitflap_driver
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set1/splitflap_driver.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +12V --power-nets-widths 0.4 0.4 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/splitflap_driver
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/splitflap_driver
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +12V --power-nets-widths 0.4 0.4 0.5
