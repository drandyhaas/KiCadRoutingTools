#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/crazyflie_fpga_deck.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 +1V0 +1V8 VBUS --plane-layers In1.Cu In2.Cu In2.Cu In2.Cu In2.Cu --via-size 0.45 --via-drill 0.3 --track-width 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb step2a_fanout_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+3V3' '!+1V0' '!+1V8' '!VBUS' --track-width 0.1 --clearance 0.1 --via-size 0.45 --via-drill 0.3 --escape-method dogbone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+3V3' '!+1V0' '!+1V8' '!VBUS' --track-width 0.1 --clearance 0.1 --via-size 0.45 --via-drill 0.3 --escape-method dogbone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1_v2.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+3V3' '!+1V0' '!+1V8' '!VBUS' --track-width 0.1 --clearance 0.1 --via-size 0.40 --via-drill 0.30 --escape-method dogbone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2a_fanout_u1_v2.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1_v3.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+3V3' '!+1V0' '!+1V8' '!VBUS' --track-width 0.08 --clearance 0.1 --via-size 0.40 --via-drill 0.30 --escape-method dogbone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2a_fanout_u1_v3.kicad_pcb step2b_capclear_v2.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_capclear_v2.kicad_pcb step3_fanout_u2.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' '!+1V0' '!+1V8' '!VBUS' --width 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_capclear_v2.kicad_pcb --output step3_fanout_u2.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' '!+1V0' '!+1V8' '!VBUS' --width 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step3_fanout_u2.kicad_pcb step3b_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3b_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3 --power-nets GND +3V3 +1V0 +1V8 VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.4 --grid-step 0.05 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3b_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3 --power-nets GND +3V3 +1V0 +1V8 VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.4 --grid-step 0.05 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3b_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3 --power-nets GND +3V3 +1V0 +1V8 VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.4 --grid-step 0.05 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/crazyflie_fpga_deck
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3b_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3 --power-nets GND +3V3 +1V0 +1V8 VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.4 --grid-step 0.05 --hole-to-hole-clearance 0.2
