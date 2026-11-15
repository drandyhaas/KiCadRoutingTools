#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cheapmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set20/cheapmesh.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cheapmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb --pairs 'Net-(J3-D+)/Net-(J3-D-)' USB_D+/USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cheapmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cheapmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb 'Net-(J3-D+)' 'Net-(J3-D-)' USB_D+ USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cheapmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cheapmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND Vin 3.3V --power-nets-widths 0.4 0.4 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cheapmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cheapmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_route.kicad_pcb step3b_retry.kicad_pcb --nets 'Net-(AE2-A)' 'Net-(J3-D-)' --rip-existing-nets SDA 3.3V User_Key Vin SCL HT_EN --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND Vin 3.3V --power-nets-widths 0.4 0.4 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/cheapmesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3b_retry.kicad_pcb step3c_finegrid.kicad_pcb --nets 'Net-(AE2-A)' 'Net-(J3-D-)' --clearance-ceiling 0.1 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --grid-step 0.05 --power-nets GND Vin 3.3V --power-nets-widths 0.4 0.4 0.3 --hole-to-hole-clearance 0.2
