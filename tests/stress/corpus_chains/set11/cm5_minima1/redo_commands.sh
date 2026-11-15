#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set11/cm5_minima1.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.13 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_M301.kicad_pcb --component Module301 --nets '*' '!GND' '!+3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_M301.kicad_pcb --component Module301 --nets '*' '!GND' '!+3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_M301.kicad_pcb --output step2b_fanout_M302.kicad_pcb --component Module302 --nets '*' '!GND' '!+3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_M302.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb step3_diffpairs.kicad_pcb '*_P' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3 --no-gnd-vias --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/cm5_minima1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.13 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V +1V8 /USB/VBUS --power-nets-widths 0.4 0.4 0.5 0.3 0.5 --max-ripup 5 --grid-step 0.1
