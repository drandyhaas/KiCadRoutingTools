#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
# REBUILT from redo_commands.timings.jsonl after a relaunch clobbered the
# original (see run_board.sh archive-on-relaunch fix). Chain walked backward
# from the graded board (step8_retry.kicad_pcb); commands are byte-faithful re-quotes.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set13/anyradio_cm5.kicad_pcb step1_planes.kicad_pcb --nets GND GND VSYS --plane-layers In1.Cu In6.Cu In3.Cu --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_xa1.kicad_pcb --component XA1 --nets '*' '!GND' '!VSYS' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1 --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_xa1.kicad_pcb step2b_clearance_xa1.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2b_clearance_xa1.kicad_pcb --output step3_fanout_u11.kicad_pcb --component U11 --nets '*' '!GND' '!VSYS' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --escape-method auto --via-size 0.25 --via-drill 0.15 --track-width 0.1 --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_fanout_u11.kicad_pcb step3b_clearance_u11.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step3b_clearance_u11.kicad_pcb --output step4_fanout_u9.kicad_pcb --component U9 --nets '*' '!GND' '!VSYS' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --escape-method auto --via-size 0.25 --via-drill 0.15 --track-width 0.1 --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step4_fanout_u9.kicad_pcb step4b_clearance_u9.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step4b_clearance_u9.kicad_pcb --output step5_fanout_U16.kicad_pcb --component U16 --nets '*' '!GND' '!VSYS' --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5_fanout_U16.kicad_pcb --output step5_fanout_U4.kicad_pcb --component U4 --nets '*' '!GND' '!VSYS' --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5_fanout_U4.kicad_pcb --output step5_fanout_U6.kicad_pcb --component U6 --nets '*' '!GND' '!VSYS' --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5_fanout_U6.kicad_pcb --output step5_fanout_U20.kicad_pcb --component U20 --nets '*' '!GND' '!VSYS' --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5_fanout_U20.kicad_pcb --output step5_fanout_U19.kicad_pcb --component U19 --nets '*' '!GND' '!VSYS' --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5_fanout_U19.kicad_pcb --output step5_fanout_U15.kicad_pcb --component U15 --nets '*' '!GND' '!VSYS' --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step5_fanout_U15.kicad_pcb step5b_clearance_final.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step5b_clearance_final.kicad_pcb --output step6_diffpairs.kicad_pcb --nets /CHG.D+ /CHG.D- /DPHY0_C_P /DPHY0_C_N /DPHY0_D0_P /DPHY0_D0_N /DPHY0_D1_P /DPHY0_D1_N /DPHY0_D2_P /DPHY0_D2_N /DPHY0_D3_P /DPHY0_D3_N /DPHY1_C_P /DPHY1_C_N /DPHY1_D0_P /DPHY1_D0_N /DPHY1_D1_P /DPHY1_D1_N /DPHY1_D2_P /DPHY1_D2_N /DPHY1_D3_P /DPHY1_D3_N /HDMI0_CK_P /HDMI0_CK_N /HDMI0_D0_P /HDMI0_D0_N /HDMI0_D1_P /HDMI0_D1_N /HDMI0_D2_P /HDMI0_D2_N /PEC+ /PEC- /PER+ /PER- /PET+ /PET- /SS0_D+ /SS0_D- /SS0_RX+ /SS0_RX- /SS0_RX1+ /SS0_RX1- /SS0_RX2+ /SS0_RX2- /SS0_TX+ /SS0_TX- /SS0_TX1+ /SS0_TX1- /SS0_TX2+ /SS0_TX2- /SS1_D+ /SS1_D- /SS1_RX+ /SS1_RX- /SS1_TX+ /SS1_TX- /SS_D+ /SS_D- /TRD0_P /TRD0_N /TRD1_P /TRD1_N /TRD2_P /TRD2_N /TRD3_P /TRD3_N /USB_OTG+ /USB_OTG- /X1+ /X1- /X2+ /X2- /X3+ /X3- /X4+ /X4- --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_diffpairs.kicad_pcb --output step7_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.12 --via-size 0.45 --via-drill 0.2 --power-nets GND VSYS 3V3_AO 3V3_M2 5V0P_CM 5V0P_SS0 3V3P_CM VBUS_CHG SS0_VBUS --power-nets-widths 0.3 0.5 0.4 0.4 0.4 0.4 0.4 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/anyradio_cm5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_route.kicad_pcb --output step8_retry.kicad_pcb --nets /CHG.STAT /CHG.EN 5V0P_SS0 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.12 --via-size 0.45 --via-drill 0.2 --power-nets 5V0P_SS0 --power-nets-widths 0.4 --max-ripup 5 --no-bga-zone U11
