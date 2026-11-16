#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/picodvi.kicad_pcb --component U1 --output step1_qfn_fanout.kicad_pcb --clearance 0.09 --nets '*' '!GND' '!+3V3' --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/picodvi.kicad_pcb --component U1 --output step1_qfn_fanout_underpad.kicad_pcb --clearance 0.09 --nets '*' '!GND' '!+3V3' --grid-step 0.05 --escape-method underpad --via-size 0.25 --via-drill 0.15 --track-width 0.08 --allow-via-in-pad
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/picodvi.kicad_pcb --component U1 --output step1_qfn_fanout_underpad.kicad_pcb --clearance 0.09 --nets '*' '!GND' '!+3V3' --grid-step 0.05 --escape-method underpad --via-size 0.25 --via-drill 0.15 --width 0.08 --allow-via-in-pad
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/picodvi.kicad_pcb --component U1 --output step1_qfn_fanout_w08.kicad_pcb --clearance 0.09 --nets '*' '!GND' '!+3V3' --grid-step 0.05 --width 0.08
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_qfn_fanout.kicad_pcb step2_place_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_place_clearance.kicad_pcb --output step3a_diff_ucdvi.kicad_pcb --nets /uC_DVI_CK+ /uC_DVI_CK- /uC_DVI_D0+ /uC_DVI_D0- /uC_DVI_D1+ /uC_DVI_D1- /uC_DVI_D2+ /uC_DVI_D2- /USB_DX+ /USB_DX- --clearance-ceiling 0.09 --track-width 0.13 --diff-pair-gap 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3a_diff_ucdvi.kicad_pcb --output step3b_diff_dvi.kicad_pcb --nets /DVI_CK+ /DVI_CK- /DVI_D0+ /DVI_D0- /DVI_D1+ /DVI_D1- /DVI_D2+ /DVI_D2- --clearance-ceiling 0.13 --track-width 0.13 --diff-pair-gap 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3b_diff_dvi.kicad_pcb --output step3c_diff_usb.kicad_pcb --nets /USB_D+ /USB_D- --clearance-ceiling 0.13 --track-width 0.36 --diff-pair-gap 0.13 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step3c_diff_usb.kicad_pcb step4_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.15 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/picodvi
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.09 --track-width 0.13 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --layers F.Cu In1.Cu In2.Cu B.Cu --power-nets GND +3V3 +1V1 VBUS --power-nets-widths 0.4 0.4 0.3 0.4 --max-ripup 5 --no-bga-zone
