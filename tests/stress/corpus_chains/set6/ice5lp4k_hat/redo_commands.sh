#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ice5lp4k_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ice5lp4k_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ice5lp4k_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set6/ice5lp4k_hat.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.1524 --via-size 0.508 --via-drill 0.254 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ice5lp4k_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2_fanout.kicad_pcb --component U401 --nets '*' '!GND' --width 0.1524 --clearance 0.1524 --via-size 0.508 --via-drill 0.254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ice5lp4k_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2_fanout.kicad_pcb --component U401 --nets '*' '!GND' --width 0.1524 --clearance 0.1 --grid-step 0.05 --via-size 0.508 --via-drill 0.254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ice5lp4k_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2_fanout.kicad_pcb --component U401 --nets '*' '!GND' --width 0.1 --clearance 0.1 --grid-step 0.05 --via-size 0.508 --via-drill 0.254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ice5lp4k_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ice5lp4k_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ice5lp4k_hat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.1524 --via-size 0.508 --via-drill 0.254 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +1V2 /FPGA/VCCIO_2 /FPGA/VCCIO_0 /FPGA/VCCPLL +1V8 --power-nets-widths 0.25 0.25 0.25 0.25 0.25 0.25 0.25
