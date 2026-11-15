#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/esp32s3_ref
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/esp32s3_ref
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/esp32s3_ref
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/esp32s3_ref.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.5 --via-drill 0.3 --track-width 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/esp32s3_ref
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb --nets D+ D- 'Net-(P1-D+)' 'Net-(P1-D-)' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.3 --no-gnd-vias --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/esp32s3_ref
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/esp32s3_ref
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3v3 +5v VIN 'Net-(U2-VBUS)' --power-nets-widths 0.3 0.3 0.4 0.4 0.4
