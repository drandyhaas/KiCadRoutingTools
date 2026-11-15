#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/allwinner_a13_som
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/allwinner_a13_som.kicad_pcb step1_pour.kicad_pcb --nets GND +1V5 --plane-layers In1.Cu In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/allwinner_a13_som
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/allwinner_a13_som
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_pour.kicad_pcb --component U200 --nets '*' '!GND' '!+1V5' --layers F.Cu In1.Cu In2.Cu B.Cu --plane-net-layers GND:In1.Cu +1V5:In2.Cu --diff-pairs '*ck+' '*ck-' '*dqs0+' '*dqs0-' '*dqs1+' '*dqs1-' --diff-pair-gap 0.1 --clearance 0.1 --track-width 0.127 --via-size 0.42 --via-drill 0.22 --output step1b_fanout_u200.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/allwinner_a13_som
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_fanout_u200.kicad_pcb --component U100 --nets '*' '!GND' '!+1V5' --width 0.127 --clearance 0.1 --output step1c_fanout_u100.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/allwinner_a13_som
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1c_fanout_u100.kicad_pcb --component U800 --nets '*' '!GND' --width 0.127 --clearance 0.1 --output step1d_fanout_u800.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/allwinner_a13_som
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1d_fanout_u800.kicad_pcb step1e_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/allwinner_a13_som
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/allwinner_a13_som
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1e_capclear.kicad_pcb step2a_diff_ddr.kicad_pcb --nets '*ck+' '*ck-' '*dqs0+' '*dqs0-' '*dqs1+' '*dqs1-' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --impedance 100 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/allwinner_a13_som
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2a_diff_ddr.kicad_pcb step2b_diff_usb.kicad_pcb --nets '*usb-d+' '*usb-d-' USB0-D+ USB0-D- USB1-D+ USB1-D- --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --impedance 90 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/allwinner_a13_som
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/allwinner_a13_som
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_diff_usb.kicad_pcb step3_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --grid-step 0.05 --hole-to-hole-clearance 0.2 --power-nets GND +1V5 +3V3 +1V2_CPU +1V2_INT +3V0 +5V_USB VBAT +3V3_periph +Vlcd '/PMIC & power rails/usb_vbus' ACIN +3V3_RTC IPS --power-nets-widths 0.4 0.4 0.4 0.4 0.3 0.3 0.4 0.4 0.3 0.3 0.3 0.3 0.2 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0
