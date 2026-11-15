#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/muzy_zynq4.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.1016 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' --diff-pairs /HDMI_CK_P /HDMI_CK_N /HDMI_D0_P /HDMI_D0_N /HDMI_D1_P /HDMI_D1_N /HDMI_D2_P /HDMI_D2_N /USB1_DP /USB1_DN /USB2_DP /USB2_DN --diff-pair-gap 0.127 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --plane-net-layers GND:In1.Cu +3V3:In2.Cu --clearance 0.1 --track-width 0.1 --via-size 0.45212 --via-drill 0.2032 --output step2_fanout_u2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' --diff-pairs /HDMI_CK_P /HDMI_CK_N /HDMI_D0_P /HDMI_D0_N /HDMI_D1_P /HDMI_D1_N /HDMI_D2_P /HDMI_D2_N /USB1_DP /USB1_DN /USB2_DP /USB2_DN --diff-pair-gap 0.127 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --plane-net-layers GND:In1.Cu +3V3:In2.Cu --clearance 0.09 --track-width 0.0762 --via-size 0.25 --via-drill 0.15 --escape-method underpad --output step2b_fanout_u2_retry.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' --diff-pairs /HDMI_CK_P /HDMI_CK_N /HDMI_D0_P /HDMI_D0_N /HDMI_D1_P /HDMI_D1_N /HDMI_D2_P /HDMI_D2_N /USB1_DP /USB1_DN /USB2_DP /USB2_DN --diff-pair-gap 0.127 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --plane-net-layers GND:In1.Cu +3V3:In2.Cu --clearance 0.09 --track-width 0.0889 --via-size 0.25 --via-drill 0.15 --escape-method auto --output step2c_fanout_u2_retry2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2b_fanout_u2_retry.kicad_pcb step3_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3_clearance.kicad_pcb step4_diffpairs.kicad_pcb /HDMI_CK_P /HDMI_CK_N /HDMI_D0_P /HDMI_D0_N /HDMI_D1_P /HDMI_D1_N /HDMI_D2_P /HDMI_D2_N /USB1_DP /USB1_DN /USB1t_DP /USB1t_DN /USB2_DP /USB2_DN /USB2t_DP /USB2t_DN --track-width 0.127 --diff-pair-gap 0.127 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.45212 --via-drill 0.2032 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1016 --hole-to-hole-clearance 0.2 --via-size 0.45212 --via-drill 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1016 --hole-to-hole-clearance 0.2 --via-size 0.45212 --via-drill 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1016 --hole-to-hole-clearance 0.2 --via-size 0.45212 --via-drill 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1016 --hole-to-hole-clearance 0.2 --via-size 0.45212 --via-drill 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1016 --hole-to-hole-clearance 0.2 --via-size 0.45212 --via-drill 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1016 --hole-to-hole-clearance 0.2 --via-size 0.45212 --via-drill 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq4
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_route.kicad_pcb step6_retry_jtag.kicad_pcb --nets JTAG_TDI JTAG_TDO --force-reroute --rip-existing-nets '*' --grid-step 0.05 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --track-width 0.0762 --via-size 0.25 --via-drill 0.15 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --max-ripup 8
