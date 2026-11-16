#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/eis.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.2 --via-size 0.5 --via-drill 0.3 --output step1_fanout_U1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_fanout_U1.kicad_pcb step1b_clearance_U1.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1b_clearance_U1.kicad_pcb --component U2 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.2 --via-size 0.5 --via-drill 0.3 --output step2_fanout_U2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2_fanout_U2.kicad_pcb --component U3 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.2 --via-size 0.5 --via-drill 0.3 --output step3_fanout_U3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_fanout_U3.kicad_pcb --component U4 --output step4_fanout_U4.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step4_fanout_U4.kicad_pcb step4b_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4b_clearance.kicad_pcb --pairs /DDMIC_CK_P:/DDMIC_CK_N /DDMIC_D0_P:/DDMIC_D0_N /DDMIC_D1_P:/DDMIC_D1_N /DDMIC_D2_P:/DDMIC_D2_N /DDMI_CK_P:/DDMI_CK_N /DDMI_D0_P:/DDMI_D0_N /DDMI_D1_P:/DDMI_D1_N /DDMI_D2_P:/DDMI_D2_N /USB1_CONN_DP:/USB1_CONN_DN /USB1_DP:/USB1_DN /USB_CONN_DP:/USB_CONN_DN /USB_DP:/USB_DN --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --no-gnd-vias --output step5_diffpairs.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4b_clearance.kicad_pcb step5_diffpairs.kicad_pcb --nets '/DDMIC_*' '/DDMI_*' '/USB1_CONN_D*' '/USB1_D*' '/USB_CONN_D*' '/USB_D*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step5_diffpairs.kicad_pcb step6_planes.kicad_pcb --planes GND:In1.Cu /PWR3V3:In2.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step5_diffpairs.kicad_pcb step6_planes.kicad_pcb --nets GND /PWR3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_planes.kicad_pcb step7_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND /PWR3V3 --power-nets-widths 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_planes.kicad_pcb step7_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND /PWR3V3 --power-nets-widths 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_planes.kicad_pcb step7_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND /PWR3V3 --power-nets-widths 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_planes.kicad_pcb step7_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND /PWR3V3 --power-nets-widths 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_planes.kicad_pcb step7_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND /PWR3V3 --power-nets-widths 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/eis
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_route.kicad_pcb step8_retry.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND /PWR3V3 --power-nets-widths 0.3 0.3 --max-ripup 5 --no-bga-zone
