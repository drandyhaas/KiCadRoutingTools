#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/argus_solarpanel.kicad_pcb step1_planes.kicad_pcb --nets GND 3V3_P --plane-layers In2.Cu In1.Cu --clearance-ceiling 0.14986
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_u6.kicad_pcb --component U6 --nets '*' '!GND' '!3V3_P' --clearance 0.14986 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_u6.kicad_pcb --component U6 --nets '*' '!GND' '!3V3_P' --clearance 0.14986 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_u6.kicad_pcb --output step2b_u7.kicad_pcb --component U7 --nets '*' '!GND' '!3V3_P' --clearance 0.14986 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_u7.kicad_pcb --output step2c_u8.kicad_pcb --component U8 --nets '*' '!GND' '!3V3_P' --clearance 0.14986 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_u8.kicad_pcb --output step2d_u9.kicad_pcb --component U9 --nets '*' '!GND' '!3V3_P' --clearance 0.14986 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2d_u9.kicad_pcb step3_capclear.kicad_pcb --clearance 0.14986
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/argus_solarpanel
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.14986 --track-width 0.2032 --via-size 0.6096 --via-drill 0.3048 --hole-to-hole-clearance 0.2 --power-nets GND 3V3_P VBATT '/Solar Charger/MPPT/VCC' '/Solar Charger/MPPT1/VCC' '/Solar Charger/MPPT2/VCC' '/Solar Charger/VCHRG' 'Net-(U6-EXTVCC)' 'Net-(U7-EXTVCC)' 'Net-(U8-EXTVCC)' 'Net-(U2-VCC)' --power-nets-widths 0.4 0.4 0.5 0.4 0.4 0.4 0.5 0.3 0.3 0.3 0.3
