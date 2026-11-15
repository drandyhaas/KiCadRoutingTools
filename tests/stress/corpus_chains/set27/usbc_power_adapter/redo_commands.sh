#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/usbc_power_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/usbc_power_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set27/usbc_power_adapter.kicad_pcb step1_planes.kicad_pcb --nets GND +12V --plane-layers In1.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.125 --hole-to-hole-clearance 0.2 --power-nets GND +12V --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/usbc_power_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/usbc_power_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/usbc_power_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.125 --width 0.125 --grid-step 0.05 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/usbc_power_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.09 --width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/usbc_power_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step2b_decap.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/usbc_power_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/usbc_power_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2b_decap.kicad_pcb step3_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.125 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5 --power-nets GND +12V VBUS +5V +3V3 VCC --power-nets-widths 0.3 0.3 0.4 0.3 0.3 0.25 --stats
