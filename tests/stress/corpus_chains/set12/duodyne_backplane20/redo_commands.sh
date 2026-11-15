#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/duodyne_backplane20.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --zone-clearance 0.2 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3 --track-width 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --ordering bus --bus --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VCC /+12V /-12V --power-nets-widths 2.0 2.0 2.0 2.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --ordering bus --bus --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VCC /+12V /-12V --power-nets-widths 2.0 2.0 2.0 2.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route_v2.kicad_pcb --nets '*' --ordering bus --bus --layer-costs 1.0 1.0 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VCC /+12V /-12V --power-nets-widths 2.0 2.0 2.0 2.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route_v3.kicad_pcb --nets '*' --ordering bus --bus --layer-costs 1.0 1.0 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VCC /+12V /-12V --power-nets-widths 2.0 2.0 2.0 2.0 --max-ripup 3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route_v4.kicad_pcb --nets '*' --ordering bus --bus --layer-costs 1.0 1.0 --clearance-ceiling 0.2 --track-width 0.1 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VCC /+12V /-12V --power-nets-widths 2.0 2.0 2.0 2.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route_v5.kicad_pcb --nets '*' --ordering bus --bus --layer-costs 1.0 1.0 --clearance-ceiling 0.2 --track-width 0.1 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VCC /+12V /-12V --power-nets-widths 2.0 2.0 2.0 2.0 --max-ripup 1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/duodyne_backplane20
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route_v6.kicad_pcb --nets '*' --ordering bus --bus --layer-costs 1.0 1.0 --clearance-ceiling 0.2 --track-width 0.1 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VCC /+12V /-12V --power-nets-widths 2.0 2.0 2.0 2.0 --max-ripup 5
