#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cparti_fpga
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/cparti_fpga.kicad_pcb step1_planes.kicad_pcb --nets GND GNDA +3V3 --plane-layers In1.Cu In1.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.45 --via-drill 0.3 --track-width 0.15 --clearance-ceiling 0.127 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cparti_fpga
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_bga_fanout.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.12 --clearance 0.127 --via-size 0.25 --via-drill 0.15 --nets '*' '!GND' '!GNDA' '!+3V3' --escape-method auto
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cparti_fpga
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_bga_fanout.kicad_pcb step2b_clearance.kicad_pcb --clearance 0.127
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cparti_fpga
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_clearance.kicad_pcb --output step3_qfn_fanout.kicad_pcb --component U2 --width 0.15 --clearance 0.1 --nets '*' '!GND' '!GNDA' '!+3V3' --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cparti_fpga
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_qfn_fanout.kicad_pcb step4_diffpair.kicad_pcb '*USB_D*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cparti_fpga
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpair.kicad_pcb step4b_usb_se.kicad_pcb --nets /USB/USB_D+ /USB/USB_D- --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cparti_fpga
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4b_usb_se.kicad_pcb step5_route_sma.kicad_pcb --nets '*_SMA' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.35 --track-width 0.3317 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cparti_fpga
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route_sma.kicad_pcb step6_route_50ohm.kicad_pcb --nets '*_50Ohm' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.125 --track-width 0.2355 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cparti_fpga
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cparti_fpga
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_route_50ohm.kicad_pcb step7_route_final.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --max-ripup 5 --power-nets GND GNDA +3V3 +5V +1V0 +1V8 VDD +3.3VA VBUS --power-nets-widths 0.4 0.4 0.4 0.35 0.4 0.3 0.25 0.25 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cparti_fpga
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_route_final.kicad_pcb step8_retry.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.127 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --max-ripup 5 --power-nets GND GNDA +3V3 +5V +1V0 +1V8 VDD +3.3VA VBUS --power-nets-widths 0.4 0.4 0.4 0.35 0.4 0.3 0.25 0.25 0.3
