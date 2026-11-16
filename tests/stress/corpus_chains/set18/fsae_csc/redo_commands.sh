#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_csc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_csc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_csc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_csc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_csc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/fsae_csc.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.15 --zone-clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_csc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --clearance 0.15 --width 0.15 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_csc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_csc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_csc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2c_capclear.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND /VDRIVE /VREF2 /VREG --power-nets-widths 0.4 0.381 0.381 0.381
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/fsae_csc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2c_capclear.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND /VDRIVE /VREF2 /VREG --power-nets-widths 0.4 0.381 0.381 0.381
