#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set6/scalenode_cm4.kicad_pcb step1_planes.kicad_pcb --nets GND 3V3_RPi --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.125 --zone-clearance 0.125 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --track-width 0.254 --power-nets GND 3V3_RPi --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_j7.kicad_pcb --component J7 --nets '*' '!GND' '!VCC5V0' '!3V3_RPi' '!VCC1V8' --escape-method underpad --allow-via-in-pad --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_j7.kicad_pcb --component J7 --nets '*' '!GND' '!VCC5V0' '!3V3_RPi' '!VCC1V8' --escape-method underpad --allow-via-in-pad --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2a_fanout_j7.kicad_pcb step2a_clr_j7.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_clr_j7.kicad_pcb --output step2b_fanout_j8.kicad_pcb --component J8 --nets '*' '!GND' --escape-method underpad --allow-via-in-pad --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_j8.kicad_pcb step2b_clr_j8.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_clr_j8.kicad_pcb --output step2c_fanout_j10.kicad_pcb --component J10 --nets '*' '!GND' '!VCC5V0' '!3V3_OUT' --escape-method underpad --allow-via-in-pad --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2c_fanout_j10.kicad_pcb step2c_clr_j10.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_clr_j10.kicad_pcb --output step2d_fanout_u3.kicad_pcb --component U3 --escape-method stub --clearance 0.125 --via-size 0.45 --via-drill 0.2 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2d_fanout_u3.kicad_pcb step2d_clr_u3.kicad_pcb --clearance 0.125
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2d_clr_u3.kicad_pcb step3_diff.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.3 --via-drill 0.15 --no-gnd-vias --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.254 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --power-nets GND 3V3_RPi 3V3_SSD /PoE/VDD_POE_IN VCC5V0 VSS GNDD 3V3_OUT /Interfaces/VBus_Slave /PoE/VO5V VCC1V8 --power-nets-widths 0.3 0.3 0.5 0.5 0.6 0.3 0.3 0.4 0.4 0.4 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/scalenode_cm4
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.254 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --power-nets GND 3V3_RPi 3V3_SSD /PoE/VDD_POE_IN VCC5V0 VSS GNDD 3V3_OUT /Interfaces/VBus_Slave /PoE/VO5V VCC1V8 --power-nets-widths 0.3 0.3 0.5 0.5 0.6 0.3 0.3 0.4 0.4 0.4 0.3
