#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/daplink
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/daplink.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/daplink
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/daplink
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2_fanout_u2.kicad_pcb --component U2 --nets '!GND' --clearance 0.2 --width 0.25 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/daplink
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u2.kicad_pcb step3_capclear.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/daplink
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/daplink
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpair.kicad_pcb --pairs /D+ /D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/daplink
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpair.kicad_pcb /D+ /D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/daplink
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/daplink
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpair.kicad_pcb step5_signal.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.15 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3.3V +5V --power-nets-widths 0.3 0.4 0.3
