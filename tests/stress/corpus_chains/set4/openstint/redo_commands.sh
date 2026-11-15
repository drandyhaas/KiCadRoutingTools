#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/openstint.kicad_pcb --component U1 --output step1_u1_fanout.kicad_pcb --clearance 0.2 --width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/openstint.kicad_pcb --component U1 --output step1_u1_fanout.kicad_pcb --clearance 0.09 --track-width 0.08 --width 0.08 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/openstint.kicad_pcb --component U1 --output step1_u1_fanout.kicad_pcb --clearance 0.09 --width 0.08 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_u1_fanout.kicad_pcb step1b_fanout_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1b_fanout_clearance.kicad_pcb --output step2_diff.kicad_pcb --nets /A+/A- /ANT+/ANT- /GANT+/GANT- /D+/D- --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1b_fanout_clearance.kicad_pcb --output step2_diff.kicad_pcb --nets /A /ANT /GANT /D --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step2_diff.kicad_pcb step3_planes.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.45 --power-nets GND +3.3V VDDA /VIN+ /VIN- --power-nets-widths 0.5 0.5 0.3 0.5 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/openstint
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_planes.kicad_pcb --output step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.45 --board-edge-clearance 0.3 --grid-step 0.05 --power-nets GND +3.3V VDDA /VIN+ /VIN- --power-nets-widths 0.5 0.5 0.3 0.5 0.5
