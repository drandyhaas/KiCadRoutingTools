#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/upsy_desky
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/upsy_desky
# [2026-08-19] removed dead command: '--timeout' is redo_stress_test.py's flag, never a routing-tool flag -- it exited rc=2 at record time too, and under 'set -e' it killed the whole replay at this line. Superseded by a clean command in this manifest.
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/upsy_desky
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set1/upsy_desky.kicad_pcb --component U1 --clearance 0.1 --grid-step 0.05 --output step1_qfn_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/upsy_desky
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_qfn_fanout.kicad_pcb step2_place_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/upsy_desky
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/upsy_desky
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_place_clearance.kicad_pcb --nets /USB_D+ /USB_D- --clearance-ceiling 0.2 --track-width 0.2 --diff-pair-gap 0.25 --no-gnd-vias --output step3_diff_usb.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/upsy_desky
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff_usb.kicad_pcb step4_diff_usb_followup.kicad_pcb --nets /USB_D+ /USB_D- --clearance-ceiling 0.2 --track-width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/upsy_desky
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/upsy_desky
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step4_diff_usb_followup.kicad_pcb step5_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +5V VBUS --power-nets-widths 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/upsy_desky
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/upsy_desky
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_signal.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --power-nets GND +3V3 +5V VBUS --power-nets-widths 0.3 0.3 0.3 0.3
