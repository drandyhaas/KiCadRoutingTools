#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ghoul
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ghoul
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ghoul
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/ghoul.kicad_pcb step1_diff.kicad_pcb --nets D+ D- --track-width 0.13 --diff-pair-gap 0.13 --no-gnd-vias --clearance-ceiling 0.13 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ghoul
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step1_diff.kicad_pcb step2_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.13 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets +3V3 5V_Post 5V_Pre SK6803_PWR VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ghoul
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_planes.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.13 --track-width 0.13 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND +3V3 5V_Post 5V_Pre SK6803_PWR VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ghoul
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_planes.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.13 --track-width 0.13 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND +3V3 5V_Post 5V_Pre SK6803_PWR VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ghoul
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_planes.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.13 --track-width 0.13 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND +3V3 5V_Post 5V_Pre SK6803_PWR VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 --max-ripup 5 --no-bga-zone
