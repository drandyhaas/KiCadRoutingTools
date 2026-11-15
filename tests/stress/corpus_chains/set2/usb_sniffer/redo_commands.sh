#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/usb_sniffer.kicad_pcb --output step1_fanout_ic1.kicad_pcb --component IC1 --clearance 0.1 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_ic1.kicad_pcb --output step2_fanout_ic4.kicad_pcb --component IC4 --clearance 0.1 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_fanout_ic4.kicad_pcb --output step3_fanout_ic9_test.kicad_pcb --component IC9 --clearance 0.1 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_fanout_ic4.kicad_pcb --output step3_fanout_ic9.kicad_pcb --component IC9 --clearance 0.1 --width 0.12 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_fanout_ic9.kicad_pcb --output step4_diff.kicad_pcb --nets '/Host USB/DP' '/Host USB/DM' /T_USB_DP /T_USB_DM --track-width 0.254 --diff-pair-gap 0.254 --clearance-ceiling 0.1905 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step4_diff.kicad_pcb --output step5_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.45 --via-drill 0.3302 --clearance-ceiling 0.1905 --hole-to-hole-clearance 0.254 --board-edge-clearance 0.2 --power-nets +3V3 --power-nets-widths 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/usb_sniffer
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb '*' --clearance-ceiling 0.1905 --track-width 0.254 --via-size 0.45 --via-drill 0.3302 --hole-to-hole-clearance 0.254 --board-edge-clearance 0.2 --power-nets GND +3V3 --power-nets-widths 0.5 0.5 --max-ripup 5
