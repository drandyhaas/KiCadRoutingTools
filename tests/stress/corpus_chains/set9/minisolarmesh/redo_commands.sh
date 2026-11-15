#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/minisolarmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/minisolarmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/minisolarmesh.kicad_pcb step1_planes.kicad_pcb --nets GND +BATT --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +BATT -SOLAR +SOLAR -BATT 3V3 --power-nets-widths 0.4 0.4 0.4 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/minisolarmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/minisolarmesh.kicad_pcb step1_planes.kicad_pcb --nets GND +BATT --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +BATT '[-]SOLAR' +SOLAR '[-]BATT' 3V3 --power-nets-widths 0.4 0.4 0.4 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/minisolarmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/minisolarmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +BATT '[-]SOLAR' +SOLAR '[-]BATT' 3V3 --power-nets-widths 0.4 0.4 0.4 0.4 0.4 0.4
