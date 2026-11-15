#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set25/icebreaker_v10e.kicad_pcb --component U4 --output step1_fanout_u4.kicad_pcb --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_fanout_u4.kicad_pcb --component U6 --output step2_fanout_u6.kicad_pcb --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2_fanout_u6.kicad_pcb --output step3_diff_usb.kicad_pcb --nets /USB_DP /USB_DM --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.15 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff_usb.kicad_pcb step3b_diff_usb_finish.kicad_pcb --nets /USB_DM /USB_DP --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --clearance-ceiling 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py step3b_diff_usb_finish.kicad_pcb step4_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_planes.kicad_pcb final_board.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 --power-nets-widths 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py final_board.kicad_pcb step6_retry_p1a4x.kicad_pcb --nets /P1A4x --rip-existing-nets /P1A9x /P1A10x /P1A3x 'Net-(R6-Pad1)' 'Net-(R2-Pad5)' /P1A8x '/~{LEDG}' '/~{LED_RGB0}' /P1A4 /FTDI_CLK --grid-step 0.05 --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/icebreaker_v10e
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_retry_p1a4x.kicad_pcb final_board2.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 --power-nets-widths 0.4 0.4
