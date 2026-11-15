#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/rfswitch01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set17/rfswitch01.kicad_pcb step1_planes.kicad_pcb --nets GND GND --plane-layers F.Cu B.Cu --clearance-ceiling 0.2 --zone-clearance 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/rfswitch01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_rf.kicad_pcb /RF_IN_OUT /RX '/RX{slash}TX_1' '/RX{slash}TX_2' /TX 'Net-(C5-Pad1)' 'Net-(C5-Pad2)' 'Net-(C6-Pad1)' 'Net-(C7-Pad1)' 'Net-(C8-Pad1)' 'Net-(C8-Pad2)' 'Net-(C9-Pad1)' 'Net-(L3-Pad1)' --track-width 0.861 --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --layers F.Cu B.Cu --max-ripup 5 --no-bga-zone --power-nets GND --power-nets-widths 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/rfswitch01
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_rf.kicad_pcb step3_signal.kicad_pcb '*' '!/RF_IN_OUT' '!/RX' '!/RX{slash}TX_1' '!/RX{slash}TX_2' '!/TX' '!Net-(C5-Pad1)' '!Net-(C5-Pad2)' '!Net-(C6-Pad1)' '!Net-(C7-Pad1)' '!Net-(C8-Pad1)' '!Net-(C8-Pad2)' '!Net-(C9-Pad1)' '!Net-(L3-Pad1)' --track-width 0.4 --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --layers F.Cu B.Cu --max-ripup 5 --no-bga-zone --power-nets GND +3V3 --power-nets-widths 0.2 0.5
