#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_diff.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set16/spartan6_6layer.kicad_pcb step1_planes.kicad_pcb --nets GND GND VCC3V3 VCC1V2 DDR1V8 /FTDI2232H/VCC1V8 --plane-layers In1.Cu In4.Cu In2.Cu In2.Cu In2.Cu In2.Cu --via-size 0.45 --via-drill 0.25 --track-width 0.13 --clearance-ceiling 0.12 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u5.kicad_pcb --component U5 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --diff-pairs '/RAM/DDR-CK_*' --diff-pair-gap 0.1 --nets '*' '!GND' '!VCC3V3' '!VCC1V2' '!DDR1V8' --via-size 0.4 --via-drill 0.2 --track-width 0.13 --clearance 0.12
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u5.kicad_pcb --component U5 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --diff-pairs '/RAM/DDR-CK_*' --diff-pair-gap 0.1 --nets '*' '!GND' '!VCC3V3' '!VCC1V2' '!DDR1V8' --via-size 0.4 --via-drill 0.2 --track-width 0.13 --clearance 0.12
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u5_retry.kicad_pcb --component U5 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --diff-pairs '/RAM/DDR-CK_*' --diff-pair-gap 0.1 --nets '*' '!GND' '!VCC3V3' '!VCC1V2' '!DDR1V8' --via-size 0.3 --via-drill 0.15 --track-width 0.13 --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u5_retry2.kicad_pcb --component U5 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --diff-pairs '/RAM/DDR-CK_*' --diff-pair-gap 0.1 --nets '*' '!GND' '!VCC3V3' '!VCC1V2' '!DDR1V8' --via-size 0.25 --via-drill 0.15 --track-width 0.1 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2a_fanout_u5_retry.kicad_pcb step2a_fanout_u5_cap.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2a_fanout_u5_cap.kicad_pcb --output step2b_fanout_u9.kicad_pcb --component U9 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --diff-pairs '/RAM/DDR-CK_*' --diff-pair-gap 0.1 --nets '*' '!GND' '!DDR1V8' --via-size 0.3 --via-drill 0.15 --track-width 0.13 --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u9.kicad_pcb step2b_fanout_u9_cap.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_u9_cap.kicad_pcb --output step2c_fanout_u2.kicad_pcb --component U2 --width 0.13 --clearance 0.1 --nets '*' '!GND' '!VCC3V3' '!/FTDI2232H/VCC1V8'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_fanout_u2.kicad_pcb --output step3_diffpairs.kicad_pcb --nets /FTDI2232H/DP /FTDI2232H/DM /RAM/DDR-CK_P /RAM/DDR-CK_N --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.3 --via-drill 0.15 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb --output step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.12 --track-width 0.13 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --power-nets GND VCC3V3 VCC1V2 DDR1V8 /FTDI2232H/VCC1V8 VCCIN VBUS --power-nets-widths 0.25 0.4 0.3 0.3 0.25 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb --output step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.12 --track-width 0.13 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --power-nets GND VCC3V3 VCC1V2 DDR1V8 /FTDI2232H/VCC1V8 VCCIN VBUS --power-nets-widths 0.25 0.4 0.3 0.3 0.25 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/spartan6_6layer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_route.kicad_pcb --output step5_retry.kicad_pcb --nets /GPIOS/GPIO-N36 /FTDI2232H/U1-D7 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.0889 --via-size 0.3 --via-drill 0.15 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5 --no-bga-zone
