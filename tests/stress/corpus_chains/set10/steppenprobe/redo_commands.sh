#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/steppenprobe
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/steppenprobe
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/steppenprobe.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.15 --zone-clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/steppenprobe
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/steppenprobe
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/steppenprobe
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/steppenprobe
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/steppenprobe
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_capclear.kicad_pcb step3_diff.kicad_pcb --nets USB_D --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.15 --no-gnd-vias --grid-step 0.05
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/steppenprobe
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --grid-step 0.05 --power-nets GND +3V3 +1V8 VBUS +5V --power-nets-widths 0.3 0.3 0.3 0.3 0.3 --max-ripup 5 --no-bga-zone
