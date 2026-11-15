#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/cm4_underwater.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.1524 --via-size 0.508 --via-drill 0.254 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_module1.kicad_pcb --component Module1 --nets '*' '!GND' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --clearance 0.1 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_module1.kicad_pcb --output step2b_fanout_u15.kicad_pcb --component U15 --nets '*' '!GND' --clearance 0.1524 --width 0.1524 --via-size 0.508 --via-drill 0.254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_module1.kicad_pcb --output step2b_fanout_u15.kicad_pcb --component U15 --nets '*' '!GND' --clearance 0.1524 --width 0.1524 --via-size 0.508 --via-drill 0.254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_u15.kicad_pcb --output step2c_fanout_u2.kicad_pcb --component U2 --nets '*' '!GND' --clearance 0.1524 --width 0.1524 --via-size 0.508 --via-drill 0.254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fanout_u2.kicad_pcb --output step2d_fanout_u13.kicad_pcb --component U13 --nets '*' '!GND' --clearance 0.1524 --width 0.1524 --via-size 0.508 --via-drill 0.254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2d_fanout_u13.kicad_pcb step2e_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2e_capclear.kicad_pcb step3_diffpairs.kicad_pcb '*' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.508 --via-drill 0.254 --no-gnd-vias --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2e_capclear.kicad_pcb step3_diffpairs.kicad_pcb '*' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.508 --via-drill 0.254 --no-gnd-vias --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.1 --via-size 0.508 --via-drill 0.254 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5 --no-bga-zone --power-nets GND /+3.3v +3.3V /+5v /USB2-HUB/VBUS +5V +3.3VA VCC +1.8V --power-nets-widths 0.3 0.3 0.3 0.4 0.4 0.4 0.25 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater/step3_diffpairs.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater/step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --track-width 0.1 --via-size 0.508 --via-drill 0.254 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5 --no-bga-zone --power-nets GND /+3.3v +3.3V /+5v /USB2-HUB/VBUS +5V +3.3VA VCC +1.8V --power-nets-widths 0.3 0.3 0.3 0.4 0.4 0.4 0.25 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater/step4_route.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater/step5_retry.kicad_pcb --nets '/CM4 GPIO/GPIO22' '/CM4 GPIO/GPIO24' '/CM4 GPIO/GPIO27' '/CM4 High Speed/CAM_GPIO' '/CM4 GPIO/SD_CMD' --clearance-ceiling 0.1 --track-width 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 8
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater/step4_route.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater/step5_retry.kicad_pcb --nets '/CM4 GPIO/GPIO22' '/CM4 GPIO/GPIO24' '/CM4 GPIO/GPIO27' '/CM4 High Speed/CAM_GPIO' '/CM4 GPIO/SD_CMD' --clearance-ceiling 0.1 --track-width 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 8
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater/step5_retry.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set14/cm4_underwater/step6_retry2.kicad_pcb --nets '/CM4 GPIO/BT_nDis' '/CM4 GPIO/ETH_LEDG' '/CM4 GPIO/GPIO_VREF' --clearance-ceiling 0.1 --track-width 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 8
