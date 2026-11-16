#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/lora_v3
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/lora_v3
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/lora_v3
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/lora_v3
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/lora_v3.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2 --track-width 0.2 --hole-to-hole-clearance 0.25 --power-nets GND --power-nets-widths 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/lora_v3
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_ic2.kicad_pcb --component IC2 --nets '!GND' --width 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/lora_v3
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_ic2.kicad_pcb --output step2b_fanout_ic4.kicad_pcb --component IC4 --nets '!GND' --width 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/lora_v3
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_ic4.kicad_pcb step2c_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/lora_v3
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/lora_v3
# [2026-08-19] removed dead command: '--timeout' is redo_stress_test.py's flag, never a routing-tool flag -- it exited rc=2 at record time too, and under 'set -e' it killed the whole replay at this line. Superseded by a clean command in this manifest.
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/lora_v3
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_clearance.kicad_pcb step3_diffpairs.kicad_pcb --nets D+ D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.3 --no-gnd-vias --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/lora_v3
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VBAT +5V +3.3V VBUS 3V VDD-SENSORS-EEPROM VDD-Display Solar+ CHG-OUT VDD-Display-Charge VDD --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.25 0.25 0.25 0.25 0.25 0.25 0.25
