#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/meshtastic_solar
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/meshtastic_solar.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/meshtastic_solar
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' --width 0.15 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/meshtastic_solar
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' --width 0.15 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/meshtastic_solar
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step3_cap_clear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/meshtastic_solar
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/meshtastic_solar
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_cap_clear.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 /VBUS /ADC_VBUS --power-nets-widths 0.4 0.4 0.5 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu
