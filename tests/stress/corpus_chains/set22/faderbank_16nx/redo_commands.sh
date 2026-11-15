#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/faderbank_16nx.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.127 --via-size 0.6 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout_u5.kicad_pcb --component U5 --nets '*' '!GND' --clearance 0.1 --width 0.1 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u5.kicad_pcb --component U5 --nets '*' '!GND' --clearance 0.1 --width 0.1 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_u5.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpairs.kicad_pcb /USB_D+ /USB_D- 'Net-(U5-USB_DP)' 'Net-(U5-USB_DM)' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.6 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V +1V1 --power-nets-widths 0.4 0.5 0.4 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/faderbank_16nx
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V +1V1 --power-nets-widths 0.4 0.5 0.4 0.3 --max-ripup 5
