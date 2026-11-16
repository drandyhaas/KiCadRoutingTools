#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v003_usb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v003_usb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v003_usb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py ch32v003_usb.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.127 --via-size 0.4572 --via-drill 0.2032 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v003_usb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout.kicad_pcb --component U3 --nets '*' '!GND' --clearance 0.1 --track-width 0.1 --via-size 0.3 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v003_usb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component U3 --nets '*' '!GND' --clearance 0.1 --width 0.1 --via-size 0.3 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v003_usb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step2b_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v003_usb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v003_usb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_clearance.kicad_pcb --output step3_diff.kicad_pcb --nets USB_DP USB_DN --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.3 --via-drill 0.15 --grid-step 0.05 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v003_usb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ch32v003_usb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb --output step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.2032 --via-size 0.4572 --via-drill 0.2032 --hole-to-hole-clearance 0.2 --power-nets GND VBUS +3V3 --power-nets-widths 0.508 0.508 0.3 --grid-step 0.05 --max-ripup 5
