#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/bitaxe_ultra
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set32/bitaxe_ultra.kicad_pcb step1_planes.kicad_pcb --nets GND /VDD --plane-layers In1.Cu In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/bitaxe_ultra
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U14 --nets '*' '!GND' '!/VDD' --output step1b_fanout_u14.kicad_pcb --via-size 0.25 --via-drill 0.15 --track-width 0.0889 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/bitaxe_ultra
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U14 --nets '*' '!GND' '!/VDD' --output step1b_fanout_u14.kicad_pcb --via-size 0.25 --via-drill 0.15 --width 0.0889 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/bitaxe_ultra
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanout_u14.kicad_pcb step1c_capopt.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/bitaxe_ultra
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1c_capopt.kicad_pcb step2a_diff.kicad_pcb --nets '*USB_D+*' '*USB_D-*' --track-width 0.15 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/bitaxe_ultra
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.09 --track-width 0.09 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 2.5 1.0 --power-nets GND /VDD /5V /3V3 /Power/VIN /Power/OUT0 /Power/SW /BM1366/1V8 /BM1366/0V8 /BM1366/VDD1_0 /BM1366/VDD2_0 /BM1366/VDD3_0 /BM1366/VDD1_1 /BM1366/VDD2_1 /BM1366/VDD3_1 --power-nets-widths 0.3 0.4 0.4 0.3 0.4 0.4 0.5 0.3 0.3 0.25 0.25 0.25 0.25 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/bitaxe_ultra
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step2_retry.kicad_pcb --nets '*' --clearance-ceiling 0.09 --track-width 0.09 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 2.5 1.0 --power-nets GND /VDD /5V /3V3 /Power/VIN /Power/OUT0 /Power/SW /BM1366/1V8 /BM1366/0V8 /BM1366/VDD1_0 /BM1366/VDD2_0 /BM1366/VDD3_0 /BM1366/VDD1_1 /BM1366/VDD2_1 /BM1366/VDD3_1 --power-nets-widths 0.3 0.4 0.4 0.3 0.4 0.4 0.5 0.3 0.3 0.25 0.25 0.25 0.25 0.25 0.25
