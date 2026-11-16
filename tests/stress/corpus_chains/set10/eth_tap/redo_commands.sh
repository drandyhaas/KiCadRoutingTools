#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/eth_tap.kicad_pcb step1_planes.kicad_pcb --nets GND 3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.125 --zone-clearance 0.125 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/eth_tap.kicad_pcb step1_planes.kicad_pcb --nets /GND /3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.125 --zone-clearance 0.125 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb step2a_fanout_U4.kicad_pcb --component U4 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!/GND' '!/3V3' --clearance 0.125 --track-width 0.125 --via-size 0.5 --via-drill 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_U4.kicad_pcb --component U4 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!/GND' '!/3V3' --clearance 0.125 --track-width 0.125 --via-size 0.5 --via-drill 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_U4b.kicad_pcb --component U4 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!/GND' '!/3V3' --clearance 0.1 --track-width 0.1 --via-size 0.35 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2a_fanout_U4b.kicad_pcb step2a_fanout_U4c.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2a_fanout_U4c.kicad_pcb --output step2b_fanout_U13.kicad_pcb --component U13 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!/GND' '!/3V3' --clearance 0.09 --track-width 0.1 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_U13.kicad_pcb step2b_fanout_U13c.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2b_fanout_U13c.kicad_pcb --output step2c_fanout_U14.kicad_pcb --component U14 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!/GND' '!/3V3' --clearance 0.1 --track-width 0.1 --via-size 0.35 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2c_fanout_U14.kicad_pcb step2c_fanout_U14c.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fanout_U14c.kicad_pcb --output step3a_fanout_U1.kicad_pcb --component U1 --nets '*' '!/GND' '!/3V3' --clearance 0.1 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3a_fanout_U1.kicad_pcb --output step3b_fanout_U2.kicad_pcb --component U2 --nets '*' '!/GND' '!/3V3' --clearance 0.1 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3b_fanout_U2.kicad_pcb --output step3c_fanout_U5.kicad_pcb --component U5 --nets '*' '!/GND' '!/3V3' --clearance 0.1 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3c_fanout_U5.kicad_pcb --output step3d_fanout_U6.kicad_pcb --component U6 --nets '*' '!/GND' '!/3V3' --clearance 0.1 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3d_fanout_U6.kicad_pcb --output step3e_fanout_U9.kicad_pcb --component U9 --nets '*' '!/GND' '!/3V3' --clearance 0.1 --width 0.1
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3e_fanout_U9.kicad_pcb --output step4_diff.kicad_pcb --nets '*_P' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb --output step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.25 --power-nets /GND /3V3 /1V2 /5V0 /1V0 /1V8 '/Management Interface/VDDA' --power-nets-widths 0.3 0.3 0.4 0.4 0.4 0.3 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/eth_tap
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb --output step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.25 --power-nets /GND /3V3 /1V2 /5V0 /1V0 /1V8 '/Management Interface/VDDA' --power-nets-widths 0.3 0.3 0.4 0.4 0.4 0.3 0.2 --max-ripup 5
