#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mbira_esp_midi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/mbira_esp_midi.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mbira_esp_midi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mbira_esp_midi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --power-nets GND +3.3V --power-nets-widths 0.5 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mbira_esp_midi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step3_route.kicad_pcb --nets 'unconnected-(SW1-Pad1)' 'unconnected-(SW1-Pad2)' 'unconnected-(SW2-Pad1)' 'unconnected-(SW2-Pad2)' 'unconnected-(SW3-Pad1)' 'unconnected-(SW3-Pad2)' 'unconnected-(SW4-Pad1)' 'unconnected-(SW4-Pad2)' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3
