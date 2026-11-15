#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/ft2232h_jtag.kicad_pcb --component U1 --output step1_fanout_u1.kicad_pcb --clearance 0.15 --width 0.15 --extension 0.3 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/ft2232h_jtag.kicad_pcb --component U4 --output step1_fanout_u4.kicad_pcb --clearance 0.15 --width 0.15 --extension 0.3 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_fanout_u4.kicad_pcb step2_fanout_clearance.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_fanout_clearance.kicad_pcb step3_diff.kicad_pcb --diff-pair-gap 0.25 --track-width 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
# [2026-08-19] removed dead command: '--timeout' is redo_stress_test.py's flag, never a routing-tool flag -- it exited rc=2 at record time too, and under 'set -e' it killed the whole replay at this line. Superseded by a clean command in this manifest.
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_diff_followup.kicad_pcb --nets /USB_CONN_D+ /USB_CONN_D- --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step4_diff_followup.kicad_pcb step5_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND --power-nets-widths 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/ft2232h_jtag
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_final.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VBUS +3.3V +1V8 --power-nets-widths 0.4 0.5 0.3 0.25
