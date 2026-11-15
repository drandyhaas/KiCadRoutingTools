#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/din41612_backplane
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set24/din41612_backplane.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/din41612_backplane
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout.kicad_pcb -c U1 --nets '*' '!GND' --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/din41612_backplane
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/din41612_backplane
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb -c U1 --nets '*' '!GND' --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/din41612_backplane
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step3_clearance.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/din41612_backplane
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/din41612_backplane
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_clearance.kicad_pcb step4a_u1nets.kicad_pcb 'Net-(Q1-G)' 'Net-(Q2-G)' 'Net-(U1-ENABLE)' 'Net-(U1-FB_PG)' 'Net-(U1-FLB)' 'Net-(U1-OV)' 'Net-(U1-SENSE-)' 'Net-(U1-TIMER)' 'Net-(U1-TIMER_OFF)' 'Net-(U1-UV)' 'Net-(U1-VCC)' --grid-step 0.05 --clearance-ceiling 0.15 --track-width 0.13 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/din41612_backplane
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4a_u1nets.kicad_pcb step4b_signal.kicad_pcb '*' '!Net-(Q1-G)' '!Net-(Q2-G)' '!Net-(U1-ENABLE)' '!Net-(U1-FB_PG)' '!Net-(U1-FLB)' '!Net-(U1-OV)' '!Net-(U1-SENSE-)' '!Net-(U1-TIMER)' '!Net-(U1-TIMER_OFF)' '!Net-(U1-UV)' '!Net-(U1-VCC)' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND -15V +15V Earth '/Input protection (postive)/Vin' '/Input protection (negative)/Vin' --power-nets-widths 0.25 1.0 1.0 0.5 1.0 1.0 --max-ripup 5 --no-bga-zones
