#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/myra_sip_interposer.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.08 --track-width 0.08 --hole-to-hole-clearance 0.2 --zone-clearance 0.08
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb step2_u4_fanout.kicad_pcb --component U4 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.09 --track-width 0.08 --via-size 0.25 --via-drill 0.15 --nets '*' '!GND' '!+3V3' --diff-pairs '/USB_MCU*' --diff-pair-gap 0.09 --escape-method dogbone --plane-net-layers GND:In1.Cu +3V3:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_u4_fanout.kicad_pcb --component U4 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.09 --track-width 0.08 --via-size 0.25 --via-drill 0.15 --nets '*' '!GND' '!+3V3' --diff-pairs '/USB_MCU*' --diff-pair-gap 0.09 --escape-method dogbone --plane-net-layers GND:In1.Cu +3V3:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_u4_fanout.kicad_pcb step2b_u4_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_u4_capclear.kicad_pcb --output step3_u1_fanout.kicad_pcb --component U1 --nets '*' '!GND' --width 0.0762 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_u4_capclear.kicad_pcb --output step3_u1_fanout.kicad_pcb --component U1 --nets '*' '!GND' --width 0.0762 --clearance 0.09 --grid-step 0.05 --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step3_u1_fanout.kicad_pcb --output step4_u5_fanout.kicad_pcb --component U5 --nets '*' '!GND' --width 0.0762 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step3_u1_fanout.kicad_pcb --output step4_u5_fanout.kicad_pcb --component U5 --nets '*' '!GND' --width 0.0762 --clearance 0.09 --grid-step 0.05 --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step4_u5_fanout.kicad_pcb step4b_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4b_capclear.kicad_pcb step5_diffpairs.kicad_pcb '/USB*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.12 --diff-pair-gap 0.09 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --impedance 90 --no-gnd-vias --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_diffpairs.kicad_pcb step6_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.0762 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND +3V3 +5V VBUS --power-nets-widths 0.2 0.2 0.2 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/myra_sip_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_route.kicad_pcb step7_retry.kicad_pcb '/~{FRAM_CS}' /JTDO +3V3 --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.0762 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.2 --grid-step 0.025 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND +3V3 +5V VBUS --power-nets-widths 0.2 0.2 0.2 0.2
