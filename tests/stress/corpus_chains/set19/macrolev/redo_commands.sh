#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/macrolev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/macrolev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set19/macrolev.kicad_pcb step1_planes.kicad_pcb --nets GND GND --plane-layers F.Cu B.Cu --clearance-ceiling 0.195
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/macrolev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/macrolev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +3.3VA --power-nets-widths 0.3 0.5 --clearance-ceiling 0.195 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/macrolev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +3.3VA --power-nets-widths 0.3 0.5 --clearance-ceiling 0.195 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/macrolev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +3.3VA --power-nets-widths 0.3 0.5 --clearance-ceiling 0.195 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/macrolev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +3.3VA --power-nets-widths 0.3 0.5 --clearance-ceiling 0.195 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/macrolev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +3.3VA --power-nets-widths 0.3 0.5 --clearance-ceiling 0.195 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/macrolev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +3.3VA --power-nets-widths 0.3 0.5 --clearance-ceiling 0.195 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/macrolev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +3.3VA --power-nets-widths 0.3 0.5 --clearance-ceiling 0.195 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --layers F.Cu B.Cu
