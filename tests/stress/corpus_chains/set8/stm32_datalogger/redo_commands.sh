#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/stm32_datalogger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set8/stm32_datalogger.kicad_pcb step1_planes.kicad_pcb --nets GND +3.3V +3.3VA --plane-layers In1.Cu In2.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --track-width 0.25 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/stm32_datalogger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u2.kicad_pcb -c U2 --nets '*' '!GND' '!+3.3V' '!+3.3VA' --width 0.15 --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/stm32_datalogger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u2.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/stm32_datalogger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpair.kicad_pcb /USB_D+ /USB_D- --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/stm32_datalogger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpair.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.15 --grid-step 0.05 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +3.3VA VBUS /VBAT --power-nets-widths 0.3 0.3 0.2 0.3 0.2
