#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/mod_bme280
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/mod_bme280.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set3/mod_bme280/step1_planes.kicad_pcb --nets GND --layers B.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/mod_bme280
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/mod_bme280.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set3/mod_bme280/step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/mod_bme280
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set3/mod_bme280/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set3/mod_bme280/step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND 'Net-(C1-Pad1)' --power-nets-widths 0.5 0.4
