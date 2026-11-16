#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/fpga_sdram.kicad_pcb -c U1 --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --clearance 0.2 --via-size 0.5 --via-drill 0.3 --nets '*' '!GND' '!+3.3V' --diff-pairs /CLK+ /CLK- /D0+ /D0- /D1+ /D1- /D2+ /D2- --diff-pair-gap 0.25 --output step1_bga_fanout_U1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/fpga_sdram.kicad_pcb -c U1 --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --clearance 0.2 --via-size 0.25 --via-drill 0.15 --nets '*' '!GND' '!+3.3V' --diff-pairs /CLK+ /CLK- /D0+ /D0- /D1+ /D1- /D2+ /D2- --diff-pair-gap 0.25 --output step1_bga_fanout_U1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/fpga_sdram.kicad_pcb -c U1 --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.15 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --nets '*' '!GND' '!+3.3V' --diff-pairs /CLK+ /CLK- /D0+ /D0- /D1+ /D1- /D2+ /D2- --diff-pair-gap 0.2 --output step1_bga_fanout_U1_v2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_bga_fanout_U1_v2.kicad_pcb step2_capclear_U1.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_capclear_U1.kicad_pcb -c U4 --output step3_qfn_U4.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_qfn_U4.kicad_pcb -c U5 --output step4_qfn_U5.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step4_qfn_U5.kicad_pcb step5_capclear_U4U5.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step5_capclear_U4U5.kicad_pcb --output step6_diffpairs.kicad_pcb --nets /CLK+ /CLK- /D0+ /D0- /D1+ /D1- /D2+ /D2- /USB_D+ /USB_D- 'Net-(U4-D+)' 'Net-(U4-D-)' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step6_diffpairs.kicad_pcb step7_planes.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --layers F.Cu In1.Cu In2.Cu B.Cu --power-nets GND +3.3V --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/fpga_sdram
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_planes.kicad_pcb step8_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND +3.3V +5V +1V1 VCCAUX +2V5 'Net-(U4-VBUS)' --power-nets-widths 0.4 0.4 0.3 0.3 0.3 0.3 0.3 --max-ripup 5 --no-bga-zone
