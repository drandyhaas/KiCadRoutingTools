#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/stm32_rfm95_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/stm32_rfm95_lora.kicad_pcb board_step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/stm32_rfm95_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board_step1_planes.kicad_pcb board_step2_route.kicad_pcb --nets '*' --power-nets GND /3V3 VCC --power-nets-widths 0.3 0.5 0.5 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3
