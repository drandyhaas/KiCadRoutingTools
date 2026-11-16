#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/stm32h7_renode_ref.kicad_pcb -o step1a_bga_u1.kicad_pcb -c U1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.125 --clearance 0.125 --via-size 0.45 --via-drill 0.2 --nets '!GND' '!+3V3' '!/MCU/USB.D+' '!/MCU/USB.D-'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1a_bga_u1.kicad_pcb -o step1b_bga_u1_usb.kicad_pcb -c U1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.15 --clearance 0.125 --via-size 0.45 --via-drill 0.2 --nets /MCU/USB.D+ /MCU/USB.D- --diff-pairs /MCU/USB.D+ /MCU/USB.D- --diff-pair-gap 0.125 --check-for-previous
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step1b_bga_u1_usb.kicad_pcb step1c_bga_u1_capclear.kicad_pcb --clearance 0.125
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1c_bga_u1_capclear.kicad_pcb step2a_qfn_u15.kicad_pcb -c U15 --width 0.125 --clearance 0.125 --nets '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1c_bga_u1_capclear.kicad_pcb -o step2a_qfn_u15.kicad_pcb -c U15 --width 0.125 --clearance 0.125 --nets '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_qfn_u15.kicad_pcb -o step2b_qfn_u10.kicad_pcb -c U10 --width 0.125 --clearance 0.125 --nets '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_qfn_u10.kicad_pcb -o step2c_qfn_u18.kicad_pcb -c U18 --width 0.125 --clearance 0.125 --nets '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2c_qfn_u18.kicad_pcb -o step2d_qfn_u25.kicad_pcb -c U25 --width 0.125 --clearance 0.125 --nets '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2d_qfn_u25.kicad_pcb -o step2e_qfn_u2.kicad_pcb -c U2 --width 0.125 --clearance 0.125 --nets '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2e_qfn_u2.kicad_pcb -o step2f_qfn_u16.kicad_pcb -c U16 --width 0.125 --clearance 0.125 --nets '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2f_qfn_u16.kicad_pcb -o step2g_qfn_u22.kicad_pcb -c U22 --width 0.0762 --clearance 0.09 --grid-step 0.05 --nets '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2g_qfn_u22.kicad_pcb -o step2h_qfn_u8.kicad_pcb -c U8 --width 0.0762 --clearance 0.09 --grid-step 0.05 --nets '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2h_qfn_u8.kicad_pcb step2i_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2i_capclear.kicad_pcb step3a_diff_can.kicad_pcb /CAN/CAN1+ /CAN/CAN1- /CAN/CAN2+ /CAN/CAN2- --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.125 --diff-pair-gap 0.4 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3a_diff_can.kicad_pcb step3b_diff_mdi.kicad_pcb /Ethernet/RXP /Ethernet/RXN /Ethernet/TXP /Ethernet/TXN --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.15 --diff-pair-gap 0.35 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3b_diff_mdi.kicad_pcb step3c_diff_usb.kicad_pcb /MCU/USB.D+ /MCU/USB.D- '/USB-C Debug/USB.D+' '/USB-C Debug/USB.D-' '/USB-C Debug/USB.DP' '/USB-C Debug/USB.DN' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.15 --diff-pair-gap 0.125 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3c_diff_usb.kicad_pcb step3d_route_can_legs.kicad_pcb /CAN/CAN1+ /CAN/CAN1- /CAN/CAN2+ /CAN/CAN2- --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.125 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3d_route_can_legs.kicad_pcb step3e_route_mdi_legs.kicad_pcb /Ethernet/RXP /Ethernet/RXN /Ethernet/TXP /Ethernet/TXN --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3e_route_mdi_legs.kicad_pcb step3f_route_usb_legs.kicad_pcb /MCU/USB.D+ /MCU/USB.D- --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py step3f_route_usb_legs.kicad_pcb step4a_planes.kicad_pcb --nets GND GND +3V3 --plane-layers In1.Cu In4.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.125 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --power-nets GND +3V3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4a_planes.kicad_pcb final_board.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.125 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V USBC_DRP_VBUS USBC_DEBUG_VBUS /Power/+5V_Shunt --power-nets-widths 0.25 0.3 0.5 0.6 0.5 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4a_planes.kicad_pcb final_board.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.125 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V USBC_DRP_VBUS USBC_DEBUG_VBUS /Power/+5V_Shunt --power-nets-widths 0.25 0.3 0.5 0.6 0.5 0.4 --overwrite
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4a_planes.kicad_pcb final_board.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.125 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V USBC_DRP_VBUS USBC_DEBUG_VBUS /Power/+5V_Shunt --power-nets-widths 0.25 0.3 0.5 0.6 0.5 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4a_planes.kicad_pcb final_board.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.125 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V USBC_DRP_VBUS USBC_DEBUG_VBUS /Power/+5V_Shunt --power-nets-widths 0.25 0.3 0.5 0.6 0.5 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/stm32h7_renode_ref
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py final_board.kicad_pcb final_board_fix1.kicad_pcb +3V3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.3 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V USBC_DRP_VBUS USBC_DEBUG_VBUS /Power/+5V_Shunt --power-nets-widths 0.25 0.3 0.5 0.6 0.5 0.4
