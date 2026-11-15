#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/led_panel_prog.kicad_pcb --component U1 --output step1_fanout_u1.kicad_pcb --nets '*' '!GND' '!+3V3' --clearance 0.125 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_fanout_u1.kicad_pcb --component U31 --output step2_fanout_u31.kicad_pcb --nets '*' '!GND' '!+3V3' --diff-pairs /USB_FTDI/FTDI_D_P,/USB_FTDI/FTDI_D_N --diff-pair-gap 0.152 --clearance 0.125 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_fanout_u1.kicad_pcb --component U31 --output step2_fanout_u31.kicad_pcb --nets '*' '!GND' '!+3V3' --clearance 0.125 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_fanout_u1.kicad_pcb --component U31 --output step2_fanout_u31.kicad_pcb --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2_fanout_u31.kicad_pcb --component U9 --output step3_fanout_u9.kicad_pcb --nets '*' '!GND' '!+3V3' '!+1V1' --clearance 0.09 --width 0.12 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step3_fanout_u9.kicad_pcb --component U29 --output step4_fanout_u29.kicad_pcb --nets '*' '!GND' '!+1V8' --clearance 0.09 --width 0.12 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py step4_fanout_u29.kicad_pcb step5_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --track-width 0.15 --clearance-ceiling 0.125 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step5_planes.kicad_pcb --output step6_diffpairs.kicad_pcb --nets /MCU/USB.D+ /MCU/USB.D- /MCU/USB_D+ /MCU/USB_D- /USB_FTDI/FTDI_D_P /USB_FTDI/FTDI_D_N --layers F.Cu In3.Cu In4.Cu B.Cu --track-width 0.152 --diff-pair-gap 0.152 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_diffpairs.kicad_pcb --output step7_route.kicad_pcb --nets '*' --layers F.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.125 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 VBUS +1V8 +1V1 +1V2 +2V5 /USB_FTDI/VBUS_DBG /FPGA/VCC_FPGA /FPGA/VCCIO_0 /FPGA/VCCIO_1 /FPGA/VCCIO_2 /FPGA/VCC_PLL --power-nets-widths 0.4 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_diffpairs.kicad_pcb --output step7_route.kicad_pcb --nets '*' --layers F.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.125 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 VBUS +1V8 +1V1 +1V2 +2V5 /USB_FTDI/VBUS_DBG /FPGA/VCC_FPGA /FPGA/VCCIO_0 /FPGA/VCCIO_1 /FPGA/VCCIO_2 /FPGA/VCC_PLL --power-nets-widths 0.4 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_diffpairs.kicad_pcb --output step7_route.kicad_pcb --nets '*' --layers F.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.125 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 VBUS +1V8 +1V1 +1V2 +2V5 /USB_FTDI/VBUS_DBG /FPGA/VCC_FPGA /FPGA/VCCIO_0 /FPGA/VCCIO_1 /FPGA/VCCIO_2 /FPGA/VCC_PLL --power-nets-widths 0.4 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_diffpairs.kicad_pcb --output step7_route.kicad_pcb --nets '*' --layers F.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.125 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 VBUS +1V8 +1V1 +1V2 +2V5 /USB_FTDI/VBUS_DBG /FPGA/VCC_FPGA /FPGA/VCCIO_0 /FPGA/VCCIO_1 /FPGA/VCCIO_2 /FPGA/VCC_PLL --power-nets-widths 0.4 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/led_panel_prog
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_diffpairs.kicad_pcb --output step7_route.kicad_pcb --nets '*' --layers F.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.125 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 VBUS +1V8 +1V1 +1V2 +2V5 /USB_FTDI/VBUS_DBG /FPGA/VCC_FPGA /FPGA/VCCIO_0 /FPGA/VCCIO_1 /FPGA/VCCIO_2 /FPGA/VCC_PLL --power-nets-widths 0.4 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25 0.25 0.25
