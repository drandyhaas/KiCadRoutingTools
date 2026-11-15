#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/robotics_psu
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/robotics_psu
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/robotics_psu.kicad_pcb step1_planes.kicad_pcb --nets GNDREF +BATT +5V +12V +3.3V --plane-layers In1.Cu In2.Cu In2.Cu In2.Cu In2.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/robotics_psu
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/robotics_psu
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --track-width 0.2 --power-nets GNDREF +BATT +5V +12V +3.3V --power-nets-widths 0.3 0.8 0.5 0.5 0.4 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/robotics_psu
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2c_route_widepower.kicad_pcb --nets '*' --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --track-width 0.2 --power-nets GNDREF +BATT +5V +12V +3.3V --power-nets-widths 1.0 2.0 0.5 1.0 0.4 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0 --max-ripup 5
