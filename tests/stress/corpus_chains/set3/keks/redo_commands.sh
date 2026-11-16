#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/keks.kicad_pcb step1_planes.kicad_pcb --nets GND /PWR3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --zone-clearance 0.2 --hole-to-hole-clearance 0.25 --via-size 0.5 --via-drill 0.3 --power-nets GND /PWR3V3 /PWR1V2 /PWR1V8 /PWR5V0 --power-nets-widths 0.4 0.4 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb step2_fanout_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!/PWR3V3' --diff-pairs '/DVI_CK*' '/DVI_D0*' '/DVI_D1*' '/DVI_D2*' --diff-pair-gap 0.1 --clearance 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!/PWR3V3' --diff-pairs '/DVI_CK*' '/DVI_D0*' '/DVI_D1*' '/DVI_D2*' --diff-pair-gap 0.1 --clearance 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!/PWR3V3' --diff-pairs '/DVI_CK*' '/DVI_D0*' '/DVI_D1*' '/DVI_D2*' --diff-pair-gap 0.1 --clearance 0.09 --track-width 0.15 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step2b_clearance_u1.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2b_clearance_u1.kicad_pcb --output step3_fanout_u2.kicad_pcb --component U2 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!/PWR3V3' --clearance 0.09 --track-width 0.15 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step3_fanout_u2.kicad_pcb --output step4_fanout_u3.kicad_pcb --component U3 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!/PWR3V3' --clearance 0.09 --track-width 0.15 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step4_fanout_u3.kicad_pcb --output step5_fanout_u5.kicad_pcb --component U5 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!/PWR3V3' --clearance 0.09 --track-width 0.15 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5_fanout_u5.kicad_pcb --output step6_fanout_u4.kicad_pcb --component U4 --nets '*' '!GND' '!/PWR3V3' --clearance 0.09 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step6_fanout_u4.kicad_pcb step6b_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step6b_clearance.kicad_pcb --output step7_diffpairs.kicad_pcb --layers F.Cu In1.Cu In2.Cu B.Cu --nets /CK_P /CK_N /D0_P /D0_N /D1_P /D1_N /D2_P /D2_N /DVI_CK_P /DVI_CK_N /DVI_D0_P /DVI_D0_N /DVI_D1_P /DVI_D1_N /DVI_D2_P /DVI_D2_N /USB2_CONN_DP /USB2_CONN_DN /USB2_DP /USB2_DN /USB3_CONN_DP /USB3_CONN_DN /USB3_DP /USB3_DN /USB_CONN_DP /USB_CONN_DN /USB_DP /USB_DN --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.25 --via-drill 0.15 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_diffpairs.kicad_pcb --output step8_signal.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.0762 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND /PWR3V3 /PWR1V2 /PWR1V8 /PWR5V0 --power-nets-widths 0.4 0.4 0.3 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_diffpairs.kicad_pcb --output step8_signal.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.0762 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND /PWR3V3 /PWR1V2 /PWR1V8 /PWR5V0 --power-nets-widths 0.4 0.4 0.3 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_diffpairs.kicad_pcb --output step8_signal.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.0762 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND /PWR3V3 /PWR1V2 /PWR1V8 /PWR5V0 --power-nets-widths 0.4 0.4 0.3 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/keks
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step8_signal.kicad_pcb --output step9_retry.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.09 --track-width 0.0762 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.25 --power-nets GND /PWR3V3 /PWR1V2 /PWR1V8 /PWR5V0 --power-nets-widths 0.4 0.4 0.3 0.3 0.3 --max-ripup 5 --no-bga-zone --grid-step 0.05
