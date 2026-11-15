#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/divtiesus_zx
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/divtiesus_zx
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set20/divtiesus_zx.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.16 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/divtiesus_zx
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/divtiesus_zx
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' --width 0.1 --extension 0.1 --clearance 0.1 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/divtiesus_zx
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step3_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/divtiesus_zx
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/divtiesus_zx
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/divtiesus_zx
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_clearance.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.13 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V --power-nets-widths 0.16 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/divtiesus_zx
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_clearance.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.13 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V --power-nets-widths 0.16 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/divtiesus_zx
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_clearance.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.16 --track-width 0.16 --grid-step 0.1 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V --power-nets-widths 0.16 0.25 0.25 --max-ripup 5 --no-bga-zone
