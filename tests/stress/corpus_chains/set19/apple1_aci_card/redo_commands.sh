#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/apple1_aci_card
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/apple1_aci_card
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set19/apple1_aci_card.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/apple1_aci_card
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/apple1_aci_card
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V --power-nets-widths 0.4 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/apple1_aci_card
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step3_retry.kicad_pcb 'Net-(A6-Pad10)' 'Net-(A1-+)' 'Net-(A1--)' 'Net-(A3-A7)' --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V --power-nets-widths 0.4 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/apple1_aci_card
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/apple1_aci_card
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_retry.kicad_pcb step4_cleanup_test.kicad_pcb 'Net-(A3-A5)' 'Net-(A3-A4)' 'Net-(A3-A7)' 'Net-(A3-~{CE1})' 'Net-(A2A-C)' 'Net-(A6-Pad10)' 'Net-(A5-Pad10)' 'Net-(A5-Pad5)' --force-reroute --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/apple1_aci_card
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_cleanup_test.kicad_pcb step5_cleanup2_test.kicad_pcb 'Net-(A3-A4)' 'Net-(A3-~{CE1})' 'Net-(A5-Pad5)' --force-reroute --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/apple1_aci_card
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_retry.kicad_pcb step4_fix_a1plus.kicad_pcb 'Net-(A1-+)' --force-reroute --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5
