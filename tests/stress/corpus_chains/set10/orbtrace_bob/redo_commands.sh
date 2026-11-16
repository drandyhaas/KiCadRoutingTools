#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/orbtrace_bob.kicad_pcb --output step1_fanout_U1.kicad_pcb --component U1 --width 0.2 --extension 0.3 --clearance 0.09 --grid-step 0.1 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/orbtrace_bob.kicad_pcb --output step1_fanout_U1.kicad_pcb --component U1 --width 0.2 --extension 0.3 --clearance 0.05 --grid-step 0.05 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/orbtrace_bob.kicad_pcb --output step1_fanout_U1.kicad_pcb --component U1 --width 0.2 --extension 0.3 --clearance 0.09 --grid-step 0.1 --escape-method underpad --via-size 0.45 --via-drill 0.25 --allow-via-in-pad --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/orbtrace_bob.kicad_pcb --output step1_fanout_U1.kicad_pcb --component U1 --width 0.15 --extension 0.3 --clearance 0.09 --grid-step 0.05 --escape-method underpad --via-size 0.25 --via-drill 0.15 --allow-via-in-pad --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_U1.kicad_pcb --output step2_fanout_U5.kicad_pcb --component U5 --width 0.2 --extension 0.3 --clearance 0.09 --grid-step 0.1 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_U1.kicad_pcb --output step2_fanout_U5.kicad_pcb --component U5 --width 0.15 --extension 0.3 --clearance 0.09 --grid-step 0.05 --escape-method underpad --via-size 0.25 --via-drill 0.15 --allow-via-in-pad --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_fanout_U5.kicad_pcb --output step3_fanout_U6.kicad_pcb --component U6 --width 0.15 --extension 0.3 --clearance 0.09 --grid-step 0.05 --escape-method underpad --via-size 0.25 --via-drill 0.15 --allow-via-in-pad --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_fanout_U6.kicad_pcb --output step4_fanout_U7.kicad_pcb --component U7 --width 0.15 --extension 0.3 --clearance 0.09 --grid-step 0.05 --escape-method underpad --via-size 0.25 --via-drill 0.15 --allow-via-in-pad --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step4_fanout_U7.kicad_pcb --output step5_fanout_U8.kicad_pcb --component U8 --width 0.15 --extension 0.3 --clearance 0.09 --grid-step 0.05 --escape-method underpad --via-size 0.25 --via-drill 0.15 --allow-via-in-pad --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5_fanout_U8.kicad_pcb --output step6_fanout_U9.kicad_pcb --component U9 --width 0.15 --extension 0.3 --clearance 0.09 --grid-step 0.05 --escape-method underpad --via-size 0.25 --via-drill 0.15 --allow-via-in-pad --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step6_fanout_U9.kicad_pcb --output step7_fanout_U10.kicad_pcb --component U10 --width 0.2 --extension 0.3 --clearance 0.09 --grid-step 0.1 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step7_fanout_U10.kicad_pcb step8_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step8_capclear.kicad_pcb step9_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/orbtrace_bob
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step9_planes.kicad_pcb --output step10_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 VTREF V5T --power-nets-widths 0.35 0.35 0.35 0.35 --max-ripup 5
