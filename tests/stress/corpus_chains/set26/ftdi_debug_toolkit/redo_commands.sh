#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/private/tmp/wt_baseline_0809/py_router
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/ftdi_debug_toolkit.kicad_pcb --component U1 --nets '!GND' --output step1_u1_fanout.kicad_pcb --clearance 0.125 --width 0.152 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_u1_fanout.kicad_pcb --component U4 --nets '!GND' '!+3V3' --output step2_u4_fanout.kicad_pcb --clearance 0.125 --width 0.152 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2_u4_fanout.kicad_pcb --component U13 --nets '!GND' '!+3V3' --output step3_u13_fanout.kicad_pcb --clearance 0.125 --width 0.152 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step3_u13_fanout.kicad_pcb step4_declear.kicad_pcb --clearance 0.125
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4_declear.kicad_pcb step5_diff.kicad_pcb /USB_D_P /USB_D_N --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.152 --diff-pair-gap 0.152 --clearance-ceiling 0.125 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py step5_diff.kicad_pcb step6_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.125 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.2 --power-nets GND +3V3 --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_planes.kicad_pcb step7_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --grid-step 0.05 --clearance-ceiling 0.125 --track-width 0.152 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V /VCCIOA /VCCIOB +1V8 /JTAG_VCC --power-nets-widths 0.3 0.3 0.3 0.25 0.25 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_planes.kicad_pcb step7_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --grid-step 0.05 --clearance-ceiling 0.125 --track-width 0.152 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V /VCCIOA /VCCIOB +1V8 /JTAG_VCC --power-nets-widths 0.3 0.3 0.3 0.25 0.25 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_planes.kicad_pcb step7_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --grid-step 0.05 --clearance-ceiling 0.125 --track-width 0.152 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V /VCCIOA /VCCIOB +1V8 /JTAG_VCC --power-nets-widths 0.3 0.3 0.3 0.25 0.25 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_planes.kicad_pcb step7_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --grid-step 0.05 --clearance-ceiling 0.125 --track-width 0.152 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V /VCCIOA /VCCIOB +1V8 /JTAG_VCC --power-nets-widths 0.3 0.3 0.3 0.25 0.25 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_planes.kicad_pcb step7_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --grid-step 0.05 --clearance-ceiling 0.125 --track-width 0.152 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V /VCCIOA /VCCIOB +1V8 /JTAG_VCC --power-nets-widths 0.3 0.3 0.3 0.25 0.25 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step7_route.kicad_pcb step8_retry.kicad_pcb --nets /UART_1_CTS_T /UART_1_RTS_T /ADBUS5 --force-reroute --rip-existing-nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --grid-step 0.05 --clearance-ceiling 0.125 --track-width 0.152 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V /VCCIOA /VCCIOB +1V8 /JTAG_VCC --power-nets-widths 0.3 0.3 0.3 0.25 0.25 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step8_retry.kicad_pcb step9_final.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --grid-step 0.05 --clearance-ceiling 0.125 --track-width 0.152 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V /VCCIOA /VCCIOB +1V8 /JTAG_VCC --power-nets-widths 0.3 0.3 0.3 0.25 0.25 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ftdi_debug_toolkit
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step9_final.kicad_pcb step10_fix.kicad_pcb --nets /VCCIOA --force-reroute --rip-existing-nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --grid-step 0.05 --clearance-ceiling 0.125 --track-width 0.152 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V /VCCIOA /VCCIOB +1V8 /JTAG_VCC --power-nets-widths 0.3 0.3 0.3 0.25 0.25 0.25 0.25 --max-ripup 5 --no-bga-zone
