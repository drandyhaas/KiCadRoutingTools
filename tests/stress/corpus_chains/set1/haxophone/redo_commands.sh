#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/haxophone
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/haxophone
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/haxophone
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set1/haxophone.kicad_pcb board_step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/haxophone
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py board_step1_planes.kicad_pcb --component U4 --nets '*' '!GND' --clearance 0.1 --output board_step2_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/haxophone
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py board_step2_fanout.kicad_pcb board_step2b_capopt.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/haxophone
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/haxophone
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py board_step2b_capopt.kicad_pcb board_step3_diff.kicad_pcb --nets 'Net-(U4-OUTP)' 'Net-(U4-OUTN)' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/haxophone
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/haxophone
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board_step3_diff.kicad_pcb board_step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND +5V +3.3V --power-nets-widths 0.3 0.3 0.3
