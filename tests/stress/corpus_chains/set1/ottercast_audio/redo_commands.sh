#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set1/ottercast_audio.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 GNDA --plane-layers In1.Cu In2.Cu In2.Cu --via-size 0.5 --via-drill 0.3 --track-width 0.2 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
# [2026-08-19] removed dead command: '--timeout' is redo_stress_test.py's flag, never a routing-tool flag -- it exited rc=2 at record time too, and under 'set -e' it killed the whole replay at this line. Superseded by a clean command in this manifest.
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!GNDA' '!+3V3' --diff-pairs '*_P' '*_N' --diff-pair-gap 0.1 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --track-width 0.15 --plane-net-layers GND:In1.Cu +3V3:In2.Cu GNDA:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step2b_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_clearance.kicad_pcb --output step3_fanout_u3.kicad_pcb --component U3 --nets '*' '!GND' '!GNDA' '!+3V3' --width 0.15 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_fanout_u3.kicad_pcb --output step4_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' '!GNDA' '!+3V3' --width 0.15 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_fanout_u3.kicad_pcb --output step4_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' '!GNDA' '!+3V3' --width 0.1 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_fanout_u3.kicad_pcb --output step4_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' '!GNDA' '!+3V3' --width 0.1 --clearance 0.09 --grid-step 0.05 --escape-method underpad --via-size 0.35 --via-drill 0.2 --allow-via-in-pad
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step4_fanout_u6.kicad_pcb step4b_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4b_clearance.kicad_pcb step5_diffpairs.kicad_pcb --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diffpairs.kicad_pcb final_board.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets GND GNDA +3V3 +1V1 VCC-DDR +3.0VA +5V +3.3V_RTC +5V_USBOTG USB_VBUSDET +2V5 +1V2 VINT +2V8 --power-nets-widths 0.3 0.25 0.3 0.25 0.3 0.2 0.3 0.15 0.25 0.15 0.15 0.15 0.15 0.15 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diffpairs.kicad_pcb final_board.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets GND GNDA +3V3 +1V1 VCC-DDR +3.0VA +5V +3.3V_RTC +5V_USBOTG USB_VBUSDET +2V5 +1V2 VINT +2V8 --power-nets-widths 0.3 0.25 0.3 0.25 0.3 0.2 0.3 0.15 0.25 0.15 0.15 0.15 0.15 0.15 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diffpairs.kicad_pcb final_board.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets GND GNDA +3V3 +1V1 VCC-DDR +3.0VA +5V +3.3V_RTC +5V_USBOTG USB_VBUSDET +2V5 +1V2 VINT +2V8 --power-nets-widths 0.3 0.25 0.3 0.25 0.3 0.2 0.3 0.15 0.25 0.15 0.15 0.15 0.15 0.15 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diffpairs.kicad_pcb final_board.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets GND GNDA +3V3 +1V1 VCC-DDR +3.0VA +5V +3.3V_RTC +5V_USBOTG USB_VBUSDET +2V5 +1V2 VINT +2V8 --power-nets-widths 0.3 0.25 0.3 0.25 0.3 0.2 0.3 0.15 0.25 0.15 0.15 0.15 0.15 0.15 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/ottercast_audio
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diffpairs.kicad_pcb final_board.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets GND GNDA +3V3 +1V1 VCC-DDR +3.0VA +5V +3.3V_RTC +5V_USBOTG USB_VBUSDET +2V5 +1V2 VINT +2V8 --power-nets-widths 0.3 0.25 0.3 0.25 0.3 0.2 0.3 0.15 0.25 0.15 0.15 0.15 0.15 0.15 --max-ripup 5
