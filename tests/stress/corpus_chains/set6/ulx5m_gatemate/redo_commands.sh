#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ulx5m_gatemate
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set6/ulx5m_gatemate.kicad_pcb step1_planes.kicad_pcb --nets GND GND VDD_CORE VDD_CLK SDRAM_VCC +3V3 +5V +1V2 --plane-layers In1.Cu In4.Cu In3.Cu In3.Cu In3.Cu In3.Cu In3.Cu In3.Cu --power-nets GND VDD_CORE VDD_CLK SDRAM_VCC +3V3 +5V +1V2 --power-nets-widths 0.3 0.3 0.3 0.3 0.25 0.25 0.2 --via-size 0.45 --via-drill 0.2 --track-width 0.125 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ulx5m_gatemate
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u4.kicad_pcb --component U4 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' '!VDD_CORE' '!VDD_CLK' '!SDRAM_VCC' '!+3V3' '!+5V' '!+1V2' --diff-pairs '*LVDS0*' '*LVDS1*' '*PCIe_RX*' '*USB25*' '*USB_PULL*' '*DDMI0_HS*' '*PCIe_TX_BC*' '*SER_CK*' --diff-pair-gap 0.15 --track-width 0.127 --clearance 0.1 --via-size 0.35 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ulx5m_gatemate
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u4.kicad_pcb step2b_pfc_u4.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ulx5m_gatemate
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_pfc_u4.kicad_pcb --output step3_fanout_u14.kicad_pcb --component U14 --nets '*' '!GND' '!VDD_CLK' --width 0.127 --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ulx5m_gatemate
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_fanout_u14.kicad_pcb --output step4_fanout_j1.kicad_pcb --component J1 --nets '*' '!GND' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.0762 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ulx5m_gatemate
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step4_fanout_j1.kicad_pcb --output step5_fanout_j2.kicad_pcb --component J2 --nets '*' '!GND' '!+3V3' '!+5V' '!VDD_CLK' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.0762 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ulx5m_gatemate
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step5_fanout_j2.kicad_pcb step6a_diff_groupA.kicad_pcb --nets '*ETH0*' '*ETH1*' '*ETH2*' '*ETH3*' '*DDMI0*' '*DDMI1*' '*LVDS0*' '*LVDS1*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.125 --diff-pair-gap 0.15 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ulx5m_gatemate
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step6a_diff_groupA.kicad_pcb step6b_diff_groupB.kicad_pcb --nets '*PCIe_CLK*' '*PCIe_RX*' '*PCIe_TX*' '*SER_CK*' '*SER_CLK*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.125 --diff-pair-gap 0.24 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ulx5m_gatemate
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step6b_diff_groupB.kicad_pcb step6c_diff_groupC.kicad_pcb --nets '*USB3_0*' '*USB_P' '*USB_N' '*FPD1*' '*USB25*' '*USB_PULL*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.2 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/ulx5m_gatemate
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6c_diff_groupC.kicad_pcb step8_route_thin.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.0762 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND VDD_CORE VDD_CLK SDRAM_VCC +3V3 +5V +1V2 VDD_SER VDD_PLL VDD_SER_PLL 'Net-(U14-AVDDL_PLL)' --power-nets-widths 0.3 0.3 0.3 0.3 0.25 0.25 0.2 0.2 0.2 0.2 0.2 --max-ripup 5 --no-bga-zone
