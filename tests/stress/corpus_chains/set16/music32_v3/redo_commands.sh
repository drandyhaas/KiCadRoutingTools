#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/music32_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/music32_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set16/music32_v3.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/music32_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_qfn_u23.kicad_pcb --component U23 --nets '*' '!GND' '!+3V3' --width 0.15 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/music32_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_qfn_u23.kicad_pcb --component U23 --nets '*' '!GND' '!+3V3' --width 0.15 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/music32_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_qfn_u23.kicad_pcb step2b_declear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/music32_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_declear.kicad_pcb step3a_diff_mic.kicad_pcb --nets 'Net-(U23-MIC1P)' 'Net-(U23-MIC1N)' 'Net-(U23-MIC2P)' 'Net-(U23-MIC2N)' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.05 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/music32_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3a_diff_mic.kicad_pcb step3b_diff_main.kicad_pcb --nets D+ D- UART_D+ UART_D- SD_D+ SD_D- --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/music32_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3b_diff_main.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --power-nets GND +3V3 VBUS /PSU/VIN DVDD +1V8 'Net-(J2-VDD)' 'Net-(U16-AVDD)' 'Net-(U23-AVDD)' 'Net-(U13-VDD33)' 'Net-(U13-VDD18)' --power-nets-widths 0.4 0.4 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/music32_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_route.kicad_pcb step5_fix_q4.kicad_pcb --nets 'Net-(Q4A-D2-Pad7)' --rip-existing-nets 'Net-(Q4A-D2-Pad7)' --force-reroute --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --grid-step 0.05
