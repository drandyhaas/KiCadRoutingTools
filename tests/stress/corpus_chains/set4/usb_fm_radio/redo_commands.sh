#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
# [2026-08-19] removed dead command: '--timeout' is redo_stress_test.py's flag, never a routing-tool flag -- it exited rc=2 at record time too, and under 'set -e' it killed the whole replay at this line. Superseded by a clean command in this manifest.
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/usb_fm_radio.kicad_pcb --output step1_fanout_U3.kicad_pcb --component U3 --nets '*' '!GND' '!+3.3V' --clearance 0.09 --width 0.15 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_fanout_U3.kicad_pcb step1b_clearance_U3.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_clearance_U3.kicad_pcb --output step2_fanout_U4.kicad_pcb --component U4 --nets '*' '!GND' '!+3.3V' --clearance 0.09 --width 0.15 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_U4.kicad_pcb step2b_clearance_U4.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_clearance_U4.kicad_pcb step3_diffpairs.kicad_pcb USB_D+ USB_D- /USB_CONN_D+ /USB_CONN_D- --track-width 0.284 --diff-pair-gap 0.2 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step3_diffpairs.kicad_pcb step4_planes.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.09 --zone-clearance 0.09 --hole-to-hole-clearance 0.3 --via-size 0.45 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/usb_fm_radio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.09 --track-width 0.3 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.3 --board-edge-clearance 0.3 --grid-step 0.05 --power-nets GND +3.3V VBUS /FUSE /LD_IN /PWR_LED_K /FMI /ANT --power-nets-widths 0.5 0.5 0.5 0.5 0.5 0.5 0.3493 0.3493
