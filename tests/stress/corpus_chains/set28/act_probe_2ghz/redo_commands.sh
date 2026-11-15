#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set28/act_probe_2ghz.kicad_pcb step1_planes.kicad_pcb --nets GND VCC VEE GND --plane-layers In1.Cu In2.Cu In2.Cu B.Cu --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.25 --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' '!VCC' '!VEE' --clearance 0.09 --width 0.15 --via-size 0.25 --via-drill 0.15 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u1.kicad_pcb --output step2b_fanout_u3.kicad_pcb --component U3 --nets '*' '!GND' '!VCC' '!VEE' --clearance 0.09 --width 0.15 --via-size 0.25 --via-drill 0.15 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_u3.kicad_pcb --output step2c_fanout_u4.kicad_pcb --component U4 --nets '*' '!GND' '!VCC' '!VEE' --clearance 0.09 --width 0.15 --via-size 0.25 --via-drill 0.15 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2c_fanout_u4.kicad_pcb step2d_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2d_capclear.kicad_pcb step2e_impedance.kicad_pcb --nets /OUT_RF /SMA_RF --impedance 50 --layers F.Cu --clearance-ceiling 0.14 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/act_probe_2ghz
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2e_impedance.kicad_pcb step3_route.kicad_pcb --nets '*' '!/OUT_RF' '!/SMA_RF' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.3 --power-nets GND VCC VEE --power-nets-widths 0.3 0.3 0.3
