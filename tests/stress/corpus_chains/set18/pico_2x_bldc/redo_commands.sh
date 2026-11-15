#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/pico_2x_bldc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/pico_2x_bldc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/pico_2x_bldc.kicad_pcb step1_planes.kicad_pcb --nets GND /PowerM1/IGBT_GND /PowerM2/IGBT_GND /VM /PowerM2/VM2 --plane-layers In1.Cu In1.Cu In1.Cu In2.Cu In2.Cu --clearance-ceiling 0.1525
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/pico_2x_bldc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/pico_2x_bldc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --power-nets GND /PowerM1/IGBT_GND /PowerM2/IGBT_GND /VM /PowerM2/VM2 +12V --power-nets-widths 0.4 0.3 0.3 0.6 0.6 0.5 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1525 --track-width 0.1525 --via-size 0.45 --via-drill 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/pico_2x_bldc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/pico_2x_bldc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/pico_2x_bldc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/pico_2x_bldc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step2b_widen_phase.kicad_pcb --nets /U_To_Plug /V_To_Plug /W_To_Plug /PowerM2/U_To_PlugM2 /PowerM2/V_To_PlugM2 /PowerM2/W_To_PlugM2 --force-reroute --power-nets /U_To_Plug /V_To_Plug /W_To_Plug /PowerM2/U_To_PlugM2 /PowerM2/V_To_PlugM2 /PowerM2/W_To_PlugM2 --power-nets-widths 2.0 2.0 2.0 2.0 2.0 2.0 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1525 --via-size 0.45 --via-drill 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/pico_2x_bldc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_widen_phase.kicad_pcb step2c_fix_orphan.kicad_pcb --nets /U_To_Plug --force-reroute --power-nets /U_To_Plug --power-nets-widths 2.0 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1525 --via-size 0.45 --via-drill 0.2 --max-ripup 5
