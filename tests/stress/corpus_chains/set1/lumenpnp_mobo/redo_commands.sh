#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set1/lumenpnp_mobo.kicad_pcb -o step1_fanout_u5.kicad_pcb -c U5 --width 0.15 --clearance 0.09 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_u5.kicad_pcb -o step2_fanout_U6.kicad_pcb -c U6 --width 0.2 --clearance 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_fanout_U6.kicad_pcb -o step3_fanout_U7.kicad_pcb -c U7 --width 0.2 --clearance 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_fanout_U7.kicad_pcb -o step4_fanout_U8.kicad_pcb -c U8 --width 0.2 --clearance 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step4_fanout_U8.kicad_pcb -o step5_fanout_U9.kicad_pcb -c U9 --width 0.2 --clearance 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5_fanout_U9.kicad_pcb -o step6_fanout_U10.kicad_pcb -c U10 --width 0.2 --clearance 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step6_fanout_U10.kicad_pcb -o step7_fanout_U11.kicad_pcb -c U11 --width 0.2 --clearance 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step7_fanout_U11.kicad_pcb step8_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step8_capclear.kicad_pcb step9_diffpairs.kicad_pcb /Microcontroller/USB_D+ /Microcontroller/USB_D- /RS485/RS-485+ /RS485/RS-485- --layers F.Cu In1.Cu In2.Cu B.Cu --diff-pair-gap 0.25 --clearance-ceiling 0.2 --track-width 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step9_diffpairs.kicad_pcb step10_diff_followup.kicad_pcb --nets /Microcontroller/USB_D+ /Microcontroller/USB_D- /RS485/RS-485+ /RS485/RS-485- --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step10_diff_followup.kicad_pcb step11_planes.kicad_pcb --nets GND VDC --plane-layers In1.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.5 --via-drill 0.3 --track-width 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --power-nets GND VDC --power-nets-widths 0.3 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/lumenpnp_mobo
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step11_planes.kicad_pcb step12_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VDC +3.3V +5V VBUS /Power/VIN --power-nets-widths 0.3 0.4 0.3 0.25 0.25 0.4
