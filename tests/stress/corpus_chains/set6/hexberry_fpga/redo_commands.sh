#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/hexberry_fpga
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/hexberry_fpga
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set6/hexberry_fpga.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers In2.Cu --via-size 0.45 --via-drill 0.3 --clearance-ceiling 0.1524 --track-width 0.1524 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/hexberry_fpga
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/hexberry_fpga
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U7 --nets '*' '!GND' --dry-run
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/hexberry_fpga
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/hexberry_fpga
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u7.kicad_pcb --component U7 --nets '*' '!GND' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.0889 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/hexberry_fpga
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/hexberry_fpga
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u7.kicad_pcb step3_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/hexberry_fpga
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/hexberry_fpga
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_capclear.kicad_pcb step4a_power_class.kicad_pcb 5V 'Net-(D1-A)' 'Net-(D1-K)' --clearance-ceiling 0.254 --track-width 0.254 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --power-nets 5V --power-nets-widths 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/hexberry_fpga
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4a_power_class.kicad_pcb step4b_main_route.kicad_pcb '*' --clearance-ceiling 0.1524 --track-width 0.1524 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --layers F.Cu In1.Cu In2.Cu B.Cu --power-nets GND VCC VCCIO1A_1B_1C 3V3 VCCIO4B VCCIO3D_3E 1V2 VCCIO3B_3C VCCIO3A VCCIO1D_1E VCCIO4A VCCA_BR VCCA_TR VCCA_TL +5V_RASPB +3V3_RASPB --power-nets-widths 0.3 0.3 0.25 0.3 0.25 0.25 0.3 0.25 0.25 0.25 0.25 0.25 0.25 0.25 0.3 0.3
