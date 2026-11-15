#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/g474_power_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set11/g474_power_charger.kicad_pcb step1_planes.kicad_pcb --nets Earth --plane-layers B.Cu --zone-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/g474_power_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/g474_power_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets Earth +18.8V +5V +BATT -BATT 'Net-(D10-K)' 'Net-(D2-K)' 'Net-(C39-Pad1)' --power-nets-widths 1.0 1.0 0.6 1.5 1.5 0.8 0.6 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/g474_power_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets Earth +18.8V +5V '?BATT' 'Net-(D10-K)' 'Net-(D2-K)' 'Net-(C39-Pad1)' --power-nets-widths 1.0 1.0 0.6 1.5 0.8 0.6 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/g474_power_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets Earth +18.8V +5V '?BATT' 'Net-(D10-K)' 'Net-(D2-K)' 'Net-(C39-Pad1)' --power-nets-widths 1.0 1.0 0.6 1.5 0.8 0.6 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/g474_power_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets Earth +18.8V +5V '?BATT' 'Net-(D10-K)' 'Net-(D2-K)' 'Net-(C39-Pad1)' --power-nets-widths 1.0 1.0 0.6 1.5 0.8 0.6 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/g474_power_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets Earth +18.8V +5V '?BATT' 'Net-(D10-K)' 'Net-(D2-K)' 'Net-(C39-Pad1)' --power-nets-widths 1.0 1.0 0.6 1.5 0.8 0.6 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/g474_power_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets Earth +18.8V +5V '?BATT' 'Net-(D10-K)' 'Net-(D2-K)' 'Net-(C39-Pad1)' --power-nets-widths 1.0 1.0 0.6 1.5 0.8 0.6 0.5
