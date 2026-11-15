#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/tx_band_splitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/tx_band_splitter.kicad_pcb step1_planes.kicad_pcb --nets GND GND --plane-layers F.Cu B.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/tx_band_splitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U104 --nets '*' '!GND' --clearance 0.1 --width 0.15 --grid-step 0.05 --output step1b_fanoutU104.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/tx_band_splitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_fanoutU104.kicad_pcb --component U107 --nets '*' '!GND' --clearance 0.1 --width 0.15 --grid-step 0.05 --output step1b_fanoutU107.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/tx_band_splitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_fanoutU107.kicad_pcb --component U106 --nets '*' '!GND' --clearance 0.1 --width 0.15 --grid-step 0.05 --output step1b_fanoutU106.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/tx_band_splitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanoutU106.kicad_pcb step1c_capopt.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/tx_band_splitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/tx_band_splitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/tx_band_splitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1c_capopt.kicad_pcb step2a_rf.kicad_pcb --nets /RF1 /RFCIN /RFOUT2 /RFOUT23 /RFOUT6 /RFOUT70 /RF_In /RF_Out_23cm /RF_Out_2m /RF_Out_6m /RF_Out_70cm 'Net-(C129-Pad2)' 'Net-(D104-A)' 'Net-(FL101-IN+)' 'Net-(FL101-OUT+)' 'Net-(FL104-IN)' 'Net-(FL104-OUT)' 'Net-(FL105-OUT)' 'Net-(FL109-IN)' 'Net-(FL109-OUT)' 'Net-(U104-RF4)' 'Net-(U106-RF1)' 'Net-(U106-RF2)' 'Net-(U107-ANT)' 'Net-(U107-RF4)' 'Net-(U109-RF_OUT)' RF_Filtered --track-width 1.08 --clearance-ceiling 0.2 --no-bga-zone --max-ripup 5 --layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/tx_band_splitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_rf.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND +11V VDD VCC /Power/+15V --power-nets-widths 0.3 1.0 0.4 0.4 0.4 --track-width 0.4 --clearance-ceiling 0.2 --no-bga-zone --max-ripup 5 --layers F.Cu B.Cu
