#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/boards_unrouted_set2
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/boards_unrouted_set2
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/boards_unrouted_set2
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/boards_unrouted_set2
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/lily58
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/lily58.kicad_pcb --component U3 --nets '*' '!GND' --clearance 0.1 --width 0.12 -o step1_fanout_U3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/lily58
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_U3.kicad_pcb --component U7 --nets '*' '!GNDA' --clearance 0.1 --width 0.12 -o step2_fanout_U7.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/lily58
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_U7.kicad_pcb step3_fanout_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/lily58
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/lily58
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_fanout_clearance.kicad_pcb step4_diffpairs.kicad_pcb --nets D+ D- D+_R D-_R 'Net-(U3-USB_DP)' 'Net-(U3-USB_DM)' 'Net-(U7-USB_DP)' 'Net-(U7-USB_DM)' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/lily58
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_signal.kicad_pcb --nets '*' --layers F.Cu B.Cu --layer-costs 1.0 1.0 --clearance-ceiling 0.2 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND GNDA +5V +5VA +3V3 +3.3VA +1V1 +1V2 VBUS --power-nets-widths 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/lily58
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_signal.kicad_pcb --nets '*' --layers F.Cu B.Cu --layer-costs 1.0 1.0 --clearance-ceiling 0.2 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND GNDA +5V +5VA +3V3 +3.3VA +1V1 +1V2 VBUS --power-nets-widths 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/lily58
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/lily58
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/lily58
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step5_signal.kicad_pcb step6_planes.kicad_pcb --nets GND GND GNDA GNDA --plane-layers F.Cu B.Cu F.Cu B.Cu --zone-clearance 0.1 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2
