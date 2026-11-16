#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set15/anyshake_explorer.kicad_pcb step1_planes.kicad_pcb --nets GNDD GND GNDA --plane-layers B.Cu B.Cu F.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u11.kicad_pcb --component U11 --width 0.127 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u11.kicad_pcb --output step2b_fanout_u16.kicad_pcb --component U16 --width 0.127 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u16.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb step3_diffpairs.kicad_pcb '/Sensors/AAF_*' '/Sensors/PREAMP_*' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone --power-nets GNDD GND GNDA '/MCU Controller/D+3V3' '/Power Supply/VCC' '/Power Supply/VIN' 'Net-(U8-VCC_RF)' 'Net-(U3-VCC)' '/Power Supply/A+2V5' '/Power Supply/A-2V5' --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.4 0.2 0.2 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone --power-nets GNDD GND GNDA '/MCU Controller/D+3V3' '/Power Supply/VCC' '/Power Supply/VIN' 'Net-(U8-VCC_RF)' 'Net-(U3-VCC)' '/Power Supply/A+2V5' '/Power Supply/A-2V5' --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.4 0.2 0.2 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/anyshake_explorer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/copy_board.py step4_route.kicad_pcb final_board.kicad_pcb
