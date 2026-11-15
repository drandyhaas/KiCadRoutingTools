#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/openairscope.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3P --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.18 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_U5.kicad_pcb --component U5 --clearance 0.1 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_U5.kicad_pcb --component U5 --clearance 0.1 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_U5.kicad_pcb --output step2b_fanout_U1.kicad_pcb --component U1 --clearance 0.1 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_U1.kicad_pcb --output step2c_fanout_U14.kicad_pcb --component U14 --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2c_fanout_U14.kicad_pcb step2d_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2d_capclear.kicad_pcb step3_diffpairs.kicad_pcb --nets /Conn/A+ /Conn/A- /Conn/B+ /Conn/B- /Conn/CAN+ /Conn/CAN- /Conn/CAN_P /Conn/CAN_N /Conn/DRS_P /Conn/DRS_N /Conn/RAK+ /Conn/RAK- /MCU/D+ /MCU/D- /MCU/M+ /MCU/M- /Sensors/L+ /Sensors/L- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/openairscope
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.18 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3P +3V3P_MCU +5VP VBUS VSUP +3V3PESP VBAT +3V3P_VDDA +3V3P_RAK /Indicators/VBAT_AN /Indicators/+5VP_AN 'Net-(U11-VDD)' --power-nets-widths 0.4 0.4 0.6 0.6 0.5 0.8 0.4 0.5 0.3 0.4 0.25 0.25 0.3 --no-bga-zone --max-ripup 5
