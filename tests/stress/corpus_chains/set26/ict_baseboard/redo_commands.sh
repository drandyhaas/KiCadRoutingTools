#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/ict_baseboard.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.125 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --track-width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_u1.kicad_pcb --component U1 --clearance 0.125 --width 0.125 --nets '*' '!GND' '!+3V3' --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_u1.kicad_pcb --component U1 --clearance 0.125 --width 0.125 --nets '*' '!GND' '!+3V3' --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_u1.kicad_pcb --output step2b_u2.kicad_pcb --component U2 --clearance 0.125 --width 0.125 --nets '*' '!GND' '!+3V3' --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_u2.kicad_pcb --output step2c_u4.kicad_pcb --component U4 --clearance 0.125 --width 0.125 --nets '*' '!GND' '!+3V3' --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2c_u4.kicad_pcb --output step2d_u8.kicad_pcb --component U8 --clearance 0.125 --width 0.125 --nets '*' '!GND' '!+3V3' --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2d_u8.kicad_pcb --output step2e_u9.kicad_pcb --component U9 --clearance 0.125 --width 0.125 --nets '*' '!GND' '!+3V3' --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2e_u9.kicad_pcb step2f_capclear.kicad_pcb --clearance 0.125
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2f_capclear.kicad_pcb --output step3_diff.kicad_pcb --nets /FTDI/USB.D+ /FTDI/USB.D- --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.18 --diff-pair-gap 0.18 --clearance-ceiling 0.125 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/ict_baseboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb --output step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.125 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --track-width 0.125 --grid-step 0.05 --max-ripup 5 --power-nets GND +3V3 VDD /Power/SoM_VDD_AON +5V_USB DUT_VDD /Power/SoM_VDD_SW /USB/+VBUS /Power/SoM_VDD_EN /Power/SoM_VDD_FB /Power/SoM_VDD_MODE1 /Power/SoM_VDD_VDD /Power/SoM_VDD_MODE2 /Power/SoM_VDD_VDRV /Power/SoM_VDD_BOOT /Power/SoM_VDD_PHASE --power-nets-widths 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2
