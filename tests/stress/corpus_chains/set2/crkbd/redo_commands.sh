#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
# [2026-08-19] removed dead command: '--timeout' is redo_stress_test.py's flag, never a routing-tool flag -- it exited rc=2 at record time too, and under 'set -e' it killed the whole replay at this line. Superseded by a clean command in this manifest.
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/crkbd.kicad_pcb -c U1 -o step1_qfn_U1.kicad_pcb --clearance 0.1 --width 0.15 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_qfn_U1.kicad_pcb -c rU1 -o step2_qfn_rU1.kicad_pcb --clearance 0.1 --width 0.15 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_qfn_rU1.kicad_pcb step3_fanout_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_fanout_clearance.kicad_pcb step4_diffpairs.kicad_pcb /left/D+ /left/D- /right/D+ /right/D- 'Net-(U1-USB_DP)' 'Net-(U1-USB_DM)' 'Net-(rU1-USB_DP)' 'Net-(rU1-USB_DM)' --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.19 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step4_diffpairs.kicad_pcb step5_planes.kicad_pcb --nets GND GNDR --plane-layers B.Cu B.Cu --clearance-ceiling 0.19 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb --nets '*' --clearance-ceiling 0.19 --hole-to-hole-clearance 0.25 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND GNDR +3V3 +3V3R +5V +5VR VBUS VBUSR +1V1 +1V1R /left/USB_VBUS_PIN /right/USB_VBUS_PIN --power-nets-widths 0.3 0.3 0.3 0.3 0.4 0.4 0.5 0.5 0.3 0.3 0.5 0.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb --nets '*' --clearance-ceiling 0.19 --hole-to-hole-clearance 0.25 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND GNDR +3V3 +3V3R +5V +5VR VBUS VBUSR +1V1 +1V1R /left/USB_VBUS_PIN /right/USB_VBUS_PIN --power-nets-widths 0.3 0.3 0.3 0.3 0.4 0.4 0.5 0.5 0.3 0.3 0.5 0.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/crkbd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_route.kicad_pcb step7_fix.kicad_pcb --nets +3V3 GND --rip-existing-nets +3V3 GND --clearance-ceiling 0.19 --hole-to-hole-clearance 0.25 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND GNDR +3V3 +3V3R +5V +5VR VBUS VBUSR +1V1 +1V1R /left/USB_VBUS_PIN /right/USB_VBUS_PIN --power-nets-widths 0.3 0.3 0.3 0.3 0.4 0.4 0.5 0.5 0.3 0.3 0.5 0.5 --max-ripup 5
