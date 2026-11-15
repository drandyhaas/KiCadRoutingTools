#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/cynthion.kicad_pcb step1_planes.kicad_pcb --nets GND GND +3V3 --plane-layers In2.Cu B.Cu In1.Cu --via-size 0.45 --via-drill 0.2 --track-width 0.127 --clearance-ceiling 0.127 --zone-clearance 0.09 --hole-to-hole-clearance 0.254 --power-nets GND +3V3 --power-nets-widths 0.3 0.3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_IC1.kicad_pcb --component IC1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' '!+3V3' --diff-pairs '*TARGET_FS_MONITOR_D*' --diff-pair-gap 0.1 --clearance 0.09 --track-width 0.1 --via-size 0.45 --via-drill 0.2 --plane-net-layers GND:In2.Cu,B.Cu +3V3:In1.Cu --escape-method auto
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_IC1.kicad_pcb step2b_clearance_IC1.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2b_clearance_IC1.kicad_pcb --output step3_fanout_U10.kicad_pcb --component U10 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' '!+3V3' --clearance 0.09 --track-width 0.1 --via-size 0.45 --via-drill 0.2 --plane-net-layers GND:In2.Cu,B.Cu +3V3:In1.Cu --escape-method auto
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_fanout_U10.kicad_pcb step3b_clearance_U10.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3b_clearance_U10.kicad_pcb --output step4_qfn_U1.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.2 --escape-method stub
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step4_qfn_U1.kicad_pcb --output step5_qfn_U2.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' --width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.2 --escape-method stub
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5_qfn_U2.kicad_pcb --output step6_qfn_U3.kicad_pcb --component U3 --nets '*' '!GND' '!+3V3' --width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.2 --escape-method stub
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step6_qfn_U3.kicad_pcb --output step7_qfn_U6.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' --width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.2 --escape-method stub
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step7_qfn_U6.kicad_pcb --output step8_qfn_U8.kicad_pcb --component U8 --nets '*' '!GND' '!+3V3' --width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.2 --escape-method stub
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step8_qfn_U8.kicad_pcb --output step9_qfn_U9.kicad_pcb --component U9 --nets '*' '!GND' '!+3V3' --width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.2 --escape-method stub
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step9_qfn_U9.kicad_pcb --output step10_qfn_U11.kicad_pcb --component U11 --nets '*' '!GND' '!+3V3' --width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.2 --escape-method stub
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step10_qfn_U11.kicad_pcb --output step11_qfn_U12.kicad_pcb --component U12 --nets '*' '!GND' '!+3V3' --width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.2 --escape-method stub
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step11_qfn_U12.kicad_pcb --output step12_qfn_U13.kicad_pcb --component U13 --nets '*' '!GND' '!+3V3' --width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.2 --escape-method stub
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step12_qfn_U13.kicad_pcb --output step13_qfn_U14.kicad_pcb --component U14 --nets '*' '!GND' '!+3V3' --width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.2 --escape-method stub
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step13_qfn_U14.kicad_pcb --output step14_qfn_U16.kicad_pcb --component U16 --nets '*' '!GND' '!+3V3' --width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.2 --escape-method stub
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step14_qfn_U16.kicad_pcb step15_diffpairs.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.254 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step15_diffpairs.kicad_pcb step16_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.127 --clearance-ceiling 0.127 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.254 --power-nets GND +3V3 VCCRAM +1V1 +5V +2V5 CONTROL_VBUS_IN AUX_VBUS_IN AUX_VBUS CONTROL_VBUS TARGET_C_VBUS TARGET_A_VBUS_IN TARGET_A_VBUS TARGET_C_VBUS_IN --power-nets-widths 0.3 0.3 0.25 0.25 0.4 0.2 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step15_diffpairs.kicad_pcb step16_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.127 --clearance-ceiling 0.127 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.254 --power-nets GND +3V3 VCCRAM +1V1 +5V +2V5 CONTROL_VBUS_IN AUX_VBUS_IN AUX_VBUS CONTROL_VBUS TARGET_C_VBUS TARGET_A_VBUS_IN TARGET_A_VBUS TARGET_C_VBUS_IN --power-nets-widths 0.3 0.3 0.25 0.25 0.4 0.2 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/cynthion
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step16_route.kicad_pcb step17_retry.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.127 --clearance-ceiling 0.127 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.254 --power-nets GND +3V3 VCCRAM +1V1 +5V +2V5 CONTROL_VBUS_IN AUX_VBUS_IN AUX_VBUS CONTROL_VBUS TARGET_C_VBUS TARGET_A_VBUS_IN TARGET_A_VBUS TARGET_C_VBUS_IN --power-nets-widths 0.3 0.3 0.25 0.25 0.4 0.2 0.4 0.4 0.4 0.4 0.4 0.4 0.4 0.4 --max-ripup 5 --no-bga-zone --rip-existing-nets '/AUX Port/AUX_TYPE_C.SCL' '/FPGA Banks 0 and 1/LED0' '/AUX Port/AUX_TYPE_C.~{INT}' '/AUX Port/AUX_TYPE_C.~{FAULT}' '/AUX Port/AUX PHY/RBIAS' TARGET_A_SENSE+ '/FPGA Banks 0 and 1/MON.GPIO' TARGET_C_SENSE+ TARGET_C_SENSE- '/FPGA Banks 0 and 1/MON.~{PWRDN}' '/TARGET Port/TARGET Type C Controller/CC1' TARGET_C_VBUS_IN TARGET_C_VBUS
