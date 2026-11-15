#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/pmw3360_mouse
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/private/tmp/wt_baseline_0809/py_router
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/private/tmp/wt_baseline_0809/py_router
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/pmw3360_mouse
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/pmw3360_mouse
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set24/pmw3360_mouse.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/pmw3360_mouse
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --output step2_fanout.kicad_pcb --nets '*' '!GND' '!+3V3' --clearance 0.2 --width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/pmw3360_mouse
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/pmw3360_mouse
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_fanout.kicad_pcb step3_u1nets.kicad_pcb --nets DPI_SWITCH LEFT_CLICK MIDDLE_CLICK MISO MOSI MOTION NCS NRST 'Net-(U1-BOOT0)' 'Net-(U1-PB2)' OSC_IN OSC_OUT RIGHT_CLICK SCK STATUS_LED SWCLK SWDIO USBDM USBDP USBDP_PULLUP --clearance-ceiling 0.10 --hole-to-hole-clearance 0.2 --grid-step 0.05 --track-width 0.15 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/pmw3360_mouse
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_u1nets.kicad_pcb step4_route.kicad_pcb --nets '*' '!DPI_SWITCH' '!LEFT_CLICK' '!MIDDLE_CLICK' '!MISO' '!MOSI' '!MOTION' '!NCS' '!NRST' '!Net-(U1-BOOT0)' '!Net-(U1-PB2)' '!OSC_IN' '!OSC_OUT' '!RIGHT_CLICK' '!SCK' '!STATUS_LED' '!SWCLK' '!SWDIO' '!USBDM' '!USBDP' '!USBDP_PULLUP' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V +2V 'Net-(U2-VDDPIX)' --power-nets-widths 0.3 0.3 0.3 0.25 0.25
