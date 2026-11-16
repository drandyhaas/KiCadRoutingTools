#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set27/arty_mpw_tester.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.6 --via-drill 0.2 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_U2.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' --clearance 0.15 --track-width 0.15 --via-size 0.6 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_U2.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' --clearance 0.15 --width 0.15 --via-size 0.6 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2a_fanout_U2.kicad_pcb step2a_clear.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_clear.kicad_pcb --output step2b_fanout_U6.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' --clearance 0.15 --width 0.15 --via-size 0.6 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2b_fanout_U6.kicad_pcb step2b_clear.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_clear.kicad_pcb --output step2c_fanout_U9.kicad_pcb --component U9 --nets '*' '!GND' '!+3V3' --clearance 0.15 --width 0.15 --via-size 0.6 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2c_fanout_U9.kicad_pcb --output step2d_fanout_U10.kicad_pcb --component U10 --nets '*' '!GND' '!+3V3' --clearance 0.15 --width 0.15 --via-size 0.6 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2d_fanout_U10.kicad_pcb step2_fanout_final.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2_fanout_final.kicad_pcb step3_diffpairs.kicad_pcb --nets 'FTDI_D*' 'USB_D*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.6 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.15 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 /5V0 VBUS /VDDIO /VCCD +1V8 /VDDA2 /VDDA1 /VDDA /VCCD1 /VCCD2 /VCCA /FTDI_3V3 --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 0.3 0.25 0.25 0.25 0.25 0.25 0.25 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.15 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --no-bga-zones J3 --power-nets GND +3V3 /5V0 VBUS /VDDIO /VCCD +1V8 /VDDA2 /VDDA1 /VDDA /VCCD1 /VCCD2 /VCCA /FTDI_3V3 --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 0.3 0.25 0.25 0.25 0.25 0.25 0.25 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/arty_mpw_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.15 --via-size 0.6 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --no-bga-zones J3 --power-nets GND +3V3 /5V0 VBUS /VDDIO /VCCD +1V8 /VDDA2 /VDDA1 /VDDA /VCCD1 /VCCD2 /VCCA /FTDI_3V3 --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 0.3 0.25 0.25 0.25 0.25 0.25 0.25 0.3
