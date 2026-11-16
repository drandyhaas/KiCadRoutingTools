#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/d1600e_psu_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/d1600e_psu_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/d1600e_psu_breakout.kicad_pcb step1_planes.kicad_pcb --nets GND GNDD VCC12V0 VCC3V3 VCC3V3_USB VCC5V0 VBUS VCC5V0_USB --plane-layers In1.Cu In1.Cu In2.Cu In2.Cu In2.Cu In2.Cu In2.Cu In2.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --via-size 0.6 --via-drill 0.2 --track-width 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/d1600e_psu_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/d1600e_psu_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U5 --nets '*' '!GND' '!GNDD' '!VCC12V0' '!VCC3V3' '!VCC3V3_USB' '!VCC5V0' '!VBUS' '!VCC5V0_USB' --output step2_fanout_u5.kicad_pcb --clearance 0.1 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/d1600e_psu_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_u5.kicad_pcb step2b_capfix.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/d1600e_psu_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2b_capfix.kicad_pcb step3_diff.kicad_pcb --nets /FTDI/USB_D+ /FTDI/USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/d1600e_psu_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/d1600e_psu_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND GNDD VCC12V0 VCC3V3 VCC3V3_USB VCC5V0 VBUS VCC5V0_USB --power-nets-widths 0.3 0.25 0.4 0.3 0.25 0.3 0.3 0.25 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0 --clearance-ceiling 0.127 --track-width 0.25 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/d1600e_psu_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_route_mp.kicad_pcb --nets 'unconnected-(J2-PadMP)' 'unconnected-(J3-PadMP)' 'unconnected-(S1-SH-PadS1)' --clearance-ceiling 0.127 --track-width 0.25 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0 --no-bga-zone
