#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/epdiy_v7.kicad_pcb step1_planes.kicad_pcb --nets GND 3.3V --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.16 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout_u3.kicad_pcb --component U3 --width 0.12 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u3.kicad_pcb --component U3 --width 0.12 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u3.kicad_pcb step3_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpairs.kicad_pcb '*USB*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --impedance 90 --clearance-ceiling 0.16 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.16 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --track-width 0.25 --power-nets GND 3.3V VIN VBUS EPD_VDD /+22V /-20V /-15V /15V /VNEG_IN --power-nets-widths 0.5 0.5 0.5 0.4 0.3 0.3 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.16 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --track-width 0.25 --power-nets GND 3.3V VIN VBUS EPD_VDD /+22V /-20V /-15V /15V /VNEG_IN --power-nets-widths 0.5 0.5 0.5 0.4 0.3 0.3 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step6_fix_usbd.kicad_pcb --nets /USB_D+ /USB_D- --force-reroute --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.16 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --track-width 0.1 --net-clearances '{"/USB_D+": 0.12, "/USB_D-": 0.12}'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/epdiy_v7
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step6_fix_usbd.kicad_pcb --nets /USB_D+ /USB_D- --force-reroute --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.16 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --track-width 0.1 --net-clearances net_clearances_usbd.json
