#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set19/endgame_trackball.kicad_pcb step1_planes.kicad_pcb --nets GND GND 1V8 --plane-layers In1.Cu In4.Cu In2.Cu --via-size 0.45 --via-drill 0.3 --clearance-ceiling 0.1016 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_mcu.kicad_pcb --component U_MCU1 --nets '*' '!GND' '!1V8' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1016 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2a_fanout_mcu.kicad_pcb step2a_fanout_mcu_capcleared.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_mcu_capcleared.kicad_pcb --output step2b_fanout_chg.kicad_pcb --component U_CHG1 --nets '*' '!GND' '!1V8' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1016 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_chg.kicad_pcb step2b_fanout_chg_capcleared.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_chg_capcleared.kicad_pcb --output step2c_fanout_mux.kicad_pcb --component U_MUX1 --nets '*' '!GND' '!1V8' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1016 --clearance 0.09 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2c_fanout_mux.kicad_pcb step2c_fanout_mux_capcleared.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_fanout_mux_capcleared.kicad_pcb step3_diffpairs.kicad_pcb --nets '*USBD_*' 'USB_D_*' --layers F.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1016 --clearance-ceiling 0.1016 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --no-gnd-vias --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signal.kicad_pcb --nets '*' --layers F.Cu In3.Cu B.Cu --track-width 0.1016 --clearance-ceiling 0.1016 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND 1V8 3V3 USB_VBUS BAT_VCC --power-nets-widths 0.3 0.3 0.2 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signal.kicad_pcb --nets '*' --layers F.Cu In3.Cu B.Cu --track-width 0.1016 --clearance-ceiling 0.1016 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND 1V8 3V3 USB_VBUS BAT_VCC --power-nets-widths 0.3 0.3 0.2 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signal.kicad_pcb --nets '*' --layers F.Cu In3.Cu B.Cu --track-width 0.1016 --clearance-ceiling 0.1016 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND 1V8 3V3 USB_VBUS BAT_VCC --power-nets-widths 0.3 0.3 0.2 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signal.kicad_pcb --nets '*' --layers F.Cu In3.Cu B.Cu --track-width 0.1016 --clearance-ceiling 0.1016 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND 1V8 3V3 USB_VBUS BAT_VCC --power-nets-widths 0.3 0.3 0.2 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signal.kicad_pcb --nets '*' --layers F.Cu In3.Cu B.Cu --track-width 0.1016 --clearance-ceiling 0.1016 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND 1V8 3V3 USB_VBUS BAT_VCC --power-nets-widths 0.3 0.3 0.2 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signal.kicad_pcb --nets '*' --layers F.Cu In3.Cu B.Cu --track-width 0.1016 --clearance-ceiling 0.1016 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND 1V8 3V3 USB_VBUS BAT_VCC --power-nets-widths 0.3 0.3 0.2 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signal.kicad_pcb --nets '*' --layers F.Cu In3.Cu B.Cu --track-width 0.1016 --clearance-ceiling 0.1016 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND 1V8 3V3 USB_VBUS BAT_VCC --power-nets-widths 0.3 0.3 0.2 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signal.kicad_pcb --nets '*' --layers F.Cu In3.Cu B.Cu --track-width 0.1016 --clearance-ceiling 0.1016 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND 1V8 3V3 USB_VBUS BAT_VCC --power-nets-widths 0.3 0.3 0.2 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_signal.kicad_pcb step5_retry.kicad_pcb --nets 'Net-(AE1-A)' VLOAD MCU_RGB_EN D_SS1_RST CHG_SCL RE2_A ARB1 --rip-existing-nets D_SS1_1V8 D_SS1_NCS D_SS1_MTN D_SS1_DIO D_SS1_GND 'Net-(U4-VCP)' D_SS1_CLK 'Net-(U4-+VCSEL)' SS1_RST SS2_CLK RE2_B SW7 RGB_DOUT SW6 SW5 RE1_B MCU_SWDIO ARB2 ARB3 SS1_DIO SS1_CLK CHG_ISACTIVE CHG_SDA 'Net-(U_CHG1-PWROK)' 'Net-(Q3-D)' SS1_NCS CHG_IRQ USB_MUX_SEL MCU_DEC46 SW5_WK SS2_NCS SS1_MTN --layers F.Cu In3.Cu B.Cu --track-width 0.0762 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND 1V8 3V3 USB_VBUS BAT_VCC --power-nets-widths 0.3 0.3 0.2 0.3 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/endgame_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_retry.kicad_pcb step6_ae1.kicad_pcb --nets 'Net-(AE1-A)' --layers F.Cu --track-width 0.0762 --clearance-ceiling 0.09 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5
