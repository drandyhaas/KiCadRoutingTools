#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/rp2040_motorctrl.kicad_pcb step1_fanout_u15.kicad_pcb --component U15 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/rp2040_motorctrl.kicad_pcb --output step1_fanout_u15.kicad_pcb --component U15 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_u15.kicad_pcb --output step2_fanout_u19.kicad_pcb --component U19 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u19.kicad_pcb step3_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpairs.kicad_pcb /CAN0_P /CAN0_N /CAN1_P /CAN1_N /USB_D+ /USB_D- /USB_DP /USB_DN /USB_D_2+ /USB_D_2- /USB_DP_2 /USB_DN_2 --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.254 --via-size 0.6 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4_diffpairs.kicad_pcb step4b_diffpair_u19usb.kicad_pcb /USB_DP_2 /USB_DN_2 --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.254 --via-size 0.6 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step4_diffpairs.kicad_pcb step5_planes.kicad_pcb --nets GND GND_ISO +3V3 --plane-layers In1.Cu In1.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.6 --via-drill 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.254 --power-nets GND GND_ISO +3V3 --power-nets-widths 0.6 0.6 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.254 --power-nets GND GND_ISO +3V3 +3.3VA +5V +12V +5V_ISO +5V_ISO_FILT VBUS VBUS2 +1V1 /VMOT_RAW /12V_switched --power-nets-widths 0.6 0.6 0.4 0.4 0.5 0.8 0.4 0.4 0.4 0.4 0.4 2.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.254 --power-nets GND GND_ISO +3V3 +3.3VA +5V +12V +5V_ISO +5V_ISO_FILT VBUS VBUS2 +1V1 /VMOT_RAW /12V_switched --power-nets-widths 0.6 0.6 0.4 0.4 0.5 0.8 0.4 0.4 0.4 0.4 0.4 2.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/rp2040_motorctrl
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.254 --power-nets GND GND_ISO +3V3 +3.3VA +5V +12V +5V_ISO +5V_ISO_FILT VBUS VBUS2 +1V1 /VMOT_RAW /12V_switched --power-nets-widths 0.6 0.6 0.4 0.4 0.5 0.8 0.4 0.4 0.4 0.4 0.4 2.0 1.0
