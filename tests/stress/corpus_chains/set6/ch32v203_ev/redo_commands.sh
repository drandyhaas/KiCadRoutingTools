#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v203_ev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v203_ev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v203_ev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v203_ev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v203_ev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set6/ch32v203_ev.kicad_pcb step1_planes.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V --power-nets-widths 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v203_ev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout.kicad_pcb --component U1 --nets '*' '!GND' '!+3.3V' --clearance 0.2 --width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v203_ev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component U1 --nets '*' '!GND' '!+3.3V' --clearance 0.2 --width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v203_ev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step2b_declearance.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v203_ev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v203_ev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_declearance.kicad_pcb step3_diffpairs.kicad_pcb --nets /USB1DP /USB1DM /USB2DP /USB2DM --layers F.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v203_ev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +5V --power-nets-widths 0.4 0.4 0.4 --max-ripup 5
