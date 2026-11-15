#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/awkb_36_split
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/awkb_36_split.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/awkb_36_split
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND VDD_5V VCC --power-nets-widths 0.5 0.5 0.5 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/awkb_36_split
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND VDD_5V VCC --power-nets-widths 0.5 0.5 0.5 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/awkb_36_split
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step3_reconnect.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND VDD_5V VCC --power-nets-widths 0.5 0.5 0.5 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/awkb_36_split
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step3_gndfix.kicad_pcb --nets GND --rip-existing-nets --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND VDD_5V VCC --power-nets-widths 0.5 0.5 0.5 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/awkb_36_split
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step3_gndfix.kicad_pcb --nets GND --rip-existing-nets GND --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND VDD_5V VCC --power-nets-widths 0.5 0.5 0.5 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/awkb_36_split
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step3_gndfix.kicad_pcb --nets GND --rip-existing-nets GND --force-reroute --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND VDD_5V VCC --power-nets-widths 0.5 0.5 0.5 --hole-to-hole-clearance 0.2
