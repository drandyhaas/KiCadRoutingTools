#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/macropad4l
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/macropad4l
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/macropad4l
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/macropad4l
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/macropad4l.kicad_pcb -o step1_fanout_U1.kicad_pcb -c U1 --clearance 0.09 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/macropad4l
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_fanout_U1.kicad_pcb step2_fanout_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/macropad4l
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/macropad4l
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_fanout_clearance.kicad_pcb step3_diff_pairs.kicad_pcb --nets /D+ --track-width 0.2 --clearance-ceiling 0.2 --diff-pair-gap 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --no-gnd-vias --layers F.Cu In1.Cu In2.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/macropad4l
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step3_diff_pairs.kicad_pcb step4_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.5 --via-drill 0.3 --track-width 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/macropad4l
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +5V --power-nets-widths 0.3 0.3 0.3 --max-ripup 5 --no-bga-zones --stats
