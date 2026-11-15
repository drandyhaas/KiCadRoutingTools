#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/eurorack_pmod
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/eurorack_pmod.kicad_pcb step1_planes.kicad_pcb --nets GND VDD --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.3 --track-width 0.152 --clearance-ceiling 0.127 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/eurorack_pmod
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/eurorack_pmod
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u1.kicad_pcb --output step2b_fanout_u7.kicad_pcb --component U7 --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/eurorack_pmod
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u7.kicad_pcb step2c_fanout_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/eurorack_pmod
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2c_fanout_clearance.kicad_pcb step3_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 -1 -1 1.0 1.0 1.0 --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND VDD +12V -12V +3.3VA 'Net-(U7-VCC)' --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/eurorack_pmod
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_route.kicad_pcb step4_retry.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 -1 -1 1.0 1.0 1.0 --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND VDD +12V -12V +3.3VA 'Net-(U7-VCC)' --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3
