#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/zynq_ad9364.kicad_pcb step1_planes.kicad_pcb --nets GND RFGND VCC_1V8 --plane-layers In1.Cu In1.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.3 --power-nets GND RFGND VCC_1V8 --power-nets-widths 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
# [2026-08-19] removed dead command: '--timeout' is redo_stress_test.py's flag, never a routing-tool flag -- it exited rc=2 at record time too, and under 'set -e' it killed the whole replay at this line. Superseded by a clean command in this manifest.
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!RFGND' '!VCC_1V8' --layers F.Cu In1.Cu In2.Cu B.Cu --diff-pairs 'DATA_CLK*' 'DDR3_CK*' --diff-pair-gap 0.15 --clearance 0.09 --via-size 0.45 --via-drill 0.3 --track-width 0.12 --output step2_u1_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!RFGND' '!VCC_1V8' --layers F.Cu In1.Cu In2.Cu B.Cu --diff-pairs 'DATA_CLK*' 'DDR3_CK*' --diff-pair-gap 0.15 --clearance 0.09 --via-size 0.45 --via-drill 0.3 --track-width 0.12 --output step2_u1_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_u1_fanout.kicad_pcb step2b_u1_clear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2b_u1_clear.kicad_pcb --component U5 --nets '*' '!GND' '!RFGND' '!VCC_1V8' --layers F.Cu In1.Cu In2.Cu B.Cu --diff-pairs 'FB_CLK*' 'RX1A*' 'RX1B*' 'RX1AN*' 'RX1BN*' 'RX_D*' 'RX_FRAME*' 'TX1A*' 'TX1B*' 'TX1AN*' 'TX1BN*' 'TX_D*' 'TX_FRAME*' 'DATA_CLK*' --diff-pair-gap 0.15 --clearance 0.09 --via-size 0.45 --via-drill 0.3 --track-width 0.12 --output step3_u5_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_u5_fanout.kicad_pcb step3b_u5_clear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step3b_u5_clear.kicad_pcb --component U2 --nets '*' '!GND' '!RFGND' '!VCC_1V8' --layers F.Cu In1.Cu In2.Cu B.Cu --diff-pairs 'DDR3_CK*' 'DDR3_DQS0*' 'DDR3_DQS1*' --diff-pair-gap 0.15 --clearance 0.09 --via-size 0.45 --via-drill 0.3 --track-width 0.12 --output step4_u2_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step4_u2_fanout.kicad_pcb step4b_u2_clear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step4b_u2_clear.kicad_pcb --component U3 --nets '*' '!GND' '!RFGND' '!VCC_1V8' --width 0.1 --clearance 0.09 --output step5_u3_fanout_v2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5_u3_fanout_v2.kicad_pcb --component U4 --nets '*' '!GND' '!RFGND' '!VCC_1V8' --width 0.1 --clearance 0.09 --output step6_u4_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step6_u4_fanout.kicad_pcb --component USD1 --nets '*' '!GND' '!RFGND' '!VCC_1V8' --width 0.1 --clearance 0.09 --output step7_usd1_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step7_usd1_fanout.kicad_pcb step8_diff.kicad_pcb --nets DATA_CLK_P DATA_CLK_N DDR3_CK_P DDR3_CK_N DDR3_DQS0_P DDR3_DQS0_N DDR3_DQS1_P DDR3_DQS1_N FB_CLK_P FB_CLK_N PHY_MDI0_P PHY_MDI0_N PHY_MDI1_P PHY_MDI1_N PHY_MDI2_P PHY_MDI2_N PHY_MDI3_P PHY_MDI3_N RX1AN_P RX1AN_N RX1A_P RX1A_N RX1BN_P RX1BN_N RX1B_P RX1B_N RX_D0_P RX_D0_N RX_D1_P RX_D1_N RX_D2_P RX_D2_N RX_D3_P RX_D3_N RX_D4_P RX_D4_N RX_D5_P RX_D5_N RX_FRAME_P RX_FRAME_N TX1AN_P TX1AN_N TX1A_P TX1A_N TX1BN_P TX1BN_N TX1B_P TX1B_N TX_D0_P TX_D0_N TX_D1_P TX_D1_N TX_D2_P TX_D2_N TX_D3_P TX_D3_N TX_D4_P TX_D4_N TX_D5_P TX_D5_N TX_FRAME_P TX_FRAME_N USB_OTG_P USB_OTG_N --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3 --no-gnd-vias --output step8_diff.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/zynq_ad9364
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step8_diff.kicad_pcb step9_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --clearance-ceiling 0.2 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND RFGND VCC_1V8 VCC_3V3 VCC_1V5 VCC_1V0 VCC_IN VDDA_RX_LO VDD_DIG ADVDD10 VCC1P8 USB_VBUS --power-nets-widths 0.4 0.4 0.4 0.3 0.3 0.3 0.4 0.25 0.25 0.25 0.25 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 6.0 2.5 1.0 --output step9_route.kicad_pcb
