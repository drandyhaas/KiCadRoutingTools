#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/teacup_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/teacup_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/teacup_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set16/teacup_carrier.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/teacup_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/teacup_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb BMC_USB_DP BMC_USB_DM MIPI0_CLKP MIPI0_CLKN MIPI0_D0P MIPI0_D0N MIPI0_D1P MIPI0_D1N MIPI1_CLKP MIPI1_CLKN MIPI1_D0P MIPI1_D0N MIPI1_D1P MIPI1_D1N USBA_HOST_DP USBA_HOST_DM --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/teacup_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --power-nets GND +3V3 +5V_SW +5V_BMC VDDR +5V_ALT VDDR_FB_SEL +3V3_ALWAYS ALTUSB_VBUS /U5_VIN DCJACK_VBUS VDDR_FB_1V5 VDDR_FB_1V35 +1V8 VDDR_FB_1V8 VDDR_FB_DIGIPOT /U1_VCC USB1_VBUS_OUT USB3_VBUS_OUT USB2_VBUS_OUT --power-nets-widths 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.5 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 --clearance-ceiling 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/teacup_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --power-nets GND +3V3 +5V_SW +5V_BMC VDDR +5V_ALT VDDR_FB_SEL +3V3_ALWAYS ALTUSB_VBUS /U5_VIN DCJACK_VBUS VDDR_FB_1V5 VDDR_FB_1V35 +1V8 VDDR_FB_1V8 VDDR_FB_DIGIPOT /U1_VCC USB1_VBUS_OUT USB3_VBUS_OUT USB2_VBUS_OUT --power-nets-widths 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.5 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 --clearance-ceiling 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/teacup_carrier
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_route.kicad_pcb step3b_retry.kicad_pcb --nets VDDR VCORE +5V_ALT /Q2_GATE --power-nets VDDR +5V_ALT --power-nets-widths 0.15 0.15 --clearance-ceiling 0.1 --track-width 0.1524 --via-size 0.35 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 8
