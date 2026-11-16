#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/hid_remapper
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/hid_remapper
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/hid_remapper.kicad_pcb -c U103 --clearance 0.1 --width 0.15 --grid-step 0.05 --via-size 0.25 --via-drill 0.15 -o step1_fanout_u103.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/hid_remapper
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_fanout_u103.kicad_pcb step2_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/hid_remapper
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/hid_remapper
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_clearance.kicad_pcb --nets '/1D+ /1D-' 'Net-(U103-USB_DP)' 'Net-(U103-USB_DM)' --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --no-gnd-vias --output step3_diffpair.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/hid_remapper
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_diffpair.kicad_pcb --nets 1D+ 1D- --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --no-gnd-vias --output step4_diffpair2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/hid_remapper
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/hid_remapper
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step4_diffpair2.kicad_pcb step5_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND --power-nets-widths 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/hid_remapper
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/hid_remapper
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +5V --power-nets-widths 0.4 0.4 0.4 0.4
