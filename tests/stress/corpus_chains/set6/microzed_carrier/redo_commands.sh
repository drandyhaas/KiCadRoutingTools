#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/microzed_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/microzed_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/microzed_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/microzed_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set6/microzed_carrier.kicad_pcb step1_planes.kicad_pcb --nets GND GND VCCIO --plane-layers In1.Cu In4.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.0889 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/microzed_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diffpairs.kicad_pcb --layers F.Cu In3.Cu B.Cu --track-width 0.122174 --diff-pair-gap 0.2032 --clearance-ceiling 0.0889 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/microzed_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diffpairs.kicad_pcb step3_signal.kicad_pcb '*' --layers F.Cu In3.Cu B.Cu --clearance-ceiling 0.0889 --track-width 0.146812 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND VCCIO +5V --power-nets-widths 0.48 0.48 0.48 --max-ripup 5
