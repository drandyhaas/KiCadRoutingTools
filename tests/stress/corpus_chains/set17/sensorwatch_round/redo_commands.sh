#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set17/sensorwatch_round.kicad_pcb step1_planes.kicad_pcb --nets GND VCC --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.0889 --zone-clearance 0.0889 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --clearance 0.0889 --width 0.0889 --via-size 0.45 --via-drill 0.2 --nets '*' '!GND' '!VCC'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step3_capclear.kicad_pcb --clearance 0.0889
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diff.kicad_pcb --nets USB_P USB_N --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.0889 --diff-pair-gap 0.0889 --clearance-ceiling 0.0889 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.0889 --track-width 0.0889 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND VCC +3V3 /VDDCORE VBUS /VDDOUT --power-nets-widths 0.2032 0.2032 0.15 0.15 0.15 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step6_retry_alarm.kicad_pcb --nets /ALARM --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.0889 --track-width 0.0762 --via-size 0.35 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/sensorwatch_round
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_retry_alarm.kicad_pcb final_board.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.0889 --track-width 0.0889 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND VCC +3V3 /VDDCORE VBUS /VDDOUT --power-nets-widths 0.2032 0.2032 0.15 0.15 0.15 0.15
