#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/fsae_custom_bms
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set11/fsae_custom_bms.kicad_pcb step1_planes.kicad_pcb --nets GND LV_GND +3.3V --plane-layers In1.Cu In1.Cu In2.Cu --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.2 --track-width 0.2 --hole-to-hole-clearance 0.2 --power-nets GND LV_GND +3.3V --power-nets-widths 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/fsae_custom_bms
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' '!LV_GND' '!+3.3V' --clearance 0.2 --width 0.2 --via-size 0.45 --via-drill 0.2 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/fsae_custom_bms
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' '!LV_GND' '!+3.3V' --clearance 0.09 --width 0.15 --via-size 0.45 --via-drill 0.2 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/fsae_custom_bms
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2a_fanout_u1.kicad_pcb step2a_fanout_u1_clr.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/fsae_custom_bms
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u1_clr.kicad_pcb --output step2b_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' '!LV_GND' '!+3.3V' --clearance 0.09 --width 0.15 --via-size 0.45 --via-drill 0.2 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/fsae_custom_bms
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u6.kicad_pcb step2b_fanout_u6_clr.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/fsae_custom_bms
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/fsae_custom_bms
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_fanout_u6_clr.kicad_pcb step3_diff.kicad_pcb --nets '*COMHP*' '*COMHN*' '*COMLP*' '*COMLN*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/fsae_custom_bms
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/fsae_custom_bms
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.1 --power-nets GND LV_GND +3.3V +12V +5V VBAT --power-nets-widths 0.4 0.4 0.4 0.6 0.4 0.5
