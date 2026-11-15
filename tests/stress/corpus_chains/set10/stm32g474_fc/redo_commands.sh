#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/stm32g474_fc.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3_CORE --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.15 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_u8.kicad_pcb --component U8 --nets '*' '!GND' '!+3V3_CORE' --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u8.kicad_pcb --component U8 --nets '*' '!GND' '!+3V3_CORE' --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u8.kicad_pcb --output step2b_fanout_u4.kicad_pcb --component U4 --nets '*' '!GND' --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u4.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb --output step3_diffpairs.kicad_pcb --nets '*USB_DP*' '*USB_DN*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/stm32g474_fc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb --output step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3_CORE +5V +5V_REG +5V_UNFLT VBUS --power-nets-widths 0.4 0.4 0.35 0.35 0.35 0.3 --max-ripup 5
