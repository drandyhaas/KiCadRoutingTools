#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/fomu
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/fomu.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.1524 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/fomu
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout.kicad_pcb --component U5 --clearance 0.1524 --track-width 0.1524 --diff-pairs /ICE_USBP /ICE_USBN --diff-pair-gap 0.0254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/fomu
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component U5 --clearance 0.1524 --width 0.1524 --nets '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/fomu
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step2b_clearance.kicad_pcb --clearance 0.1524
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/fomu
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_clearance.kicad_pcb step3_diff.kicad_pcb --pairs /ICE_USBP /ICE_USBN /PU_CTRL_USBP /PU_CTRL_USBN --track-width 0.1524 --diff-pair-gap 0.1524 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/fomu
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_clearance.kicad_pcb step3_diff.kicad_pcb /ICE_USBP /ICE_USBN /PU_CTRL_USBP /PU_CTRL_USBN --track-width 0.1524 --diff-pair-gap 0.1524 --clearance-ceiling 0.1524 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/fomu
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.1524 --via-size 0.508 --via-drill 0.254 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +5V +1V2 +2V5 /VCCPLL --power-nets-widths 0.4 0.4 0.4 0.3 0.3 0.3
