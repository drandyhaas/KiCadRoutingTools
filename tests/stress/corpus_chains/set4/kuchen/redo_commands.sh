#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/kuchen
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/kuchen.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.125 --clearance 0.125 --via-size 0.45 --via-drill 0.2 --nets '*' '!GND' '!/PWR3V3' --diff-pairs '*DVI_CK*' '*DVI_D0*' '*DVI_D1*' '*DVI_D2*' '*USBH_D?' '*USB_D?' --diff-pair-gap 0.18 --output step1_fanout_U1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/kuchen
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_fanout_U1.kicad_pcb step2_capclear.kicad_pcb --clearance 0.125
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/kuchen
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/kuchen
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_capclear.kicad_pcb --output step3_diff_hdmi.kicad_pcb /CK_P /CK_N /D0_P /D0_N /D1_P /D1_N /D2_P /D2_N /DVI_CK_P /DVI_CK_N /DVI_D0_P /DVI_D0_N /DVI_D1_P /DVI_D1_N /DVI_D2_P /DVI_D2_N --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.19 --diff-pair-gap 0.18 --clearance-ceiling 0.125 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/kuchen
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_capclear.kicad_pcb step3_diff_hdmi.kicad_pcb /CK_P /CK_N /D0_P /D0_N /D1_P /D1_N /D2_P /D2_N /DVI_CK_P /DVI_CK_N /DVI_D0_P /DVI_D0_N /DVI_D1_P /DVI_D1_N /DVI_D2_P /DVI_D2_N --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.19 --diff-pair-gap 0.18 --clearance-ceiling 0.125 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/kuchen
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_diff_hdmi.kicad_pcb step4_diff_usb.kicad_pcb /USB_DP /USB_DN /USBH_DP /USBH_DN /USB_CONN_DP /USB_CONN_DN /USBH_CONN_DP /USBH_CONN_DN --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.23 --diff-pair-gap 0.185 --clearance-ceiling 0.125 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/kuchen
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/kuchen
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step4_diff_usb.kicad_pcb step5_planes.kicad_pcb --nets GND /PWR3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/kuchen
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/kuchen
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND /PWR3V3 /PWR1V2 /PWR5V0 /PWR2V5 --power-nets-widths 0.3 0.3 0.4 0.4 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --track-width 0.125 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/kuchen
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_route.kicad_pcb step7_route_iter2.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND /PWR3V3 /PWR1V2 /PWR5V0 /PWR2V5 --power-nets-widths 0.3 0.3 0.4 0.4 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --track-width 0.125 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25
