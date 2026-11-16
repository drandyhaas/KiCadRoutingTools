#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set11/zynthian_v5_main.kicad_pcb step1_pour.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.14
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_pour.kicad_pcb --component U6 --nets '*' '!GND' --clearance 0.14 --width 0.2 --output step2a_fanout_u6.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u6.kicad_pcb --component U2 --nets '*' '!GND' --clearance 0.14 --width 0.2 --output step2b_fanout_u2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u2.kicad_pcb step2c_capopt.kicad_pcb --clearance 0.14
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capopt.kicad_pcb step3_diffpairs.kicad_pcb --nets OUTLP OUTLN OUTRP OUTRN inl+ inl- inr+ inr- outl+ outl- outr+ outr- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/zynthian_v5_main
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.14 --track-width 0.25 --via-size 0.45 --via-drill 0.24 --hole-to-hole-clearance 0.2 --grid-step 0.05 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --power-nets GND 5V DVDD 3.3V AVDD AVDD2 AVDD3 --power-nets-widths 0.3 0.4 0.3 0.3 0.3 0.3 0.3 --max-ripup 5
