#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/agri_smart_switch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/agri_smart_switch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/agri_smart_switch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set28/agri_smart_switch.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.14 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/agri_smart_switch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' --escape-method underpad --via-size 0.25 --via-drill 0.15 --clearance 0.09 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/agri_smart_switch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' --escape-method underpad --via-size 0.25 --via-drill 0.15 --clearance 0.09 --width 0.1 --allow-via-in-pad
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/agri_smart_switch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u6.kicad_pcb step3_place_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/agri_smart_switch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/agri_smart_switch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_place_clearance.kicad_pcb step4_diffpairs.kicad_pcb --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.13 --clearance-ceiling 0.13 --impedance 90 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/agri_smart_switch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/agri_smart_switch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.14 --track-width 0.2 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +5V +24V +1V1 '/sheet_4_SIM Interface/SIM_VCC' 'Net-(U2-VIN)' 'Net-(U6-ADC_AVDD)' 'Net-(U6-VREG_AVDD)' --power-nets-widths 0.5 0.35 0.35 0.6 0.25 0.25 0.3 0.2 0.2
