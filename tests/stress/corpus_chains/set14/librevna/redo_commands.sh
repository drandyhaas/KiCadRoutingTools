#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/librevna.kicad_pcb step1_planes.kicad_pcb --nets GND GND +3V3 --plane-layers In1.Cu In4.Cu In3.Cu --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_01_fanout_U2.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_01_fanout_U2.kicad_pcb --output step2_02_fanout_IC37.kicad_pcb --component IC37 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_02_fanout_IC37.kicad_pcb --output step2_03_fanout_IC21.kicad_pcb --component IC21 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_03_fanout_IC21.kicad_pcb --output step2_04_fanout_IC10.kicad_pcb --component IC10 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_04_fanout_IC10.kicad_pcb --output step2_05_fanout_IC14.kicad_pcb --component IC14 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_05_fanout_IC14.kicad_pcb --output step2_06_fanout_IC24.kicad_pcb --component IC24 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_06_fanout_IC24.kicad_pcb --output step2_07_fanout_IC29.kicad_pcb --component IC29 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_07_fanout_IC29.kicad_pcb --output step2_08_fanout_IC33.kicad_pcb --component IC33 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_08_fanout_IC33.kicad_pcb --output step2_09_fanout_IC16.kicad_pcb --component IC16 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_09_fanout_IC16.kicad_pcb --output step2_10_fanout_IC13.kicad_pcb --component IC13 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_10_fanout_IC13.kicad_pcb --output step2_11_fanout_IC18.kicad_pcb --component IC18 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_11_fanout_IC18.kicad_pcb --output step2_12_fanout_IC17.kicad_pcb --component IC17 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_12_fanout_IC17.kicad_pcb --output step2_13_fanout_IC7.kicad_pcb --component IC7 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_13_fanout_IC7.kicad_pcb --output step2_14_fanout_IC2.kicad_pcb --component IC2 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_13_fanout_IC7.kicad_pcb --output step2_14_fanout_IC2.kicad_pcb --component IC2 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_14_fanout_IC2.kicad_pcb --output step2_15_fanout_IC1.kicad_pcb --component IC1 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_15_fanout_IC1.kicad_pcb --output step2_16_fanout_IC15.kicad_pcb --component IC15 --nets '*' '!GND' '!+3V3' --clearance 0.10 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_16_fanout_IC15.kicad_pcb --output step2_17_fanout_IC11.kicad_pcb --component IC11 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.12
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_17_fanout_IC11.kicad_pcb --output step2_18_fanout_IC12.kicad_pcb --component IC12 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.12
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_16_fanout_IC15.kicad_pcb --output step2_17b_fanout_IC11.kicad_pcb --component IC11 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.12 --escape-method underpad --via-size 0.25 --via-drill 0.15 --allow-via-in-pad
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_17b_fanout_IC11.kicad_pcb --output step2_18b_fanout_IC12.kicad_pcb --component IC12 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.12 --escape-method underpad --via-size 0.25 --via-drill 0.15 --allow-via-in-pad
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_18b_fanout_IC12.kicad_pcb step3_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpairs.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_signal.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.2 --clearance-ceiling 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +6V VCCINT +5V '+5V{slash}1' '+5V{slash}2' '+5V{slash}3' '+5V{slash}4' 'Net-(IC27-VCC)' 'Net-(IC36-VCC)' 'Net-(IC22-VCC)' --power-nets-widths 0.3 0.3 0.4 0.3 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/librevna
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_signal.kicad_pcb step5b_fix.kicad_pcb '/HF Source/SRC_FILT_IN_ANT' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.12 --clearance-ceiling 0.09 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --max-ripup 5
