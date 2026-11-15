#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/kevinbot_v3.kicad_pcb step1_planes.kicad_pcb --nets GND +5V --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_u14.kicad_pcb --component U14 --nets '*' '!GND' '!+5V' --width 0.15 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u14.kicad_pcb --component U14 --nets '*' '!GND' '!+5V' --width 0.15 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u14.kicad_pcb --output step2b_fanout_u15.kicad_pcb --component U15 --nets '*' '!GND' '!+5V' --width 0.12 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u15.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb --output step3_diffpairs.kicad_pcb --nets '/RS-422/*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signals.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND +5V '/I2C Outputs/Servo Controller/VMOT' '/Power Delivery/FUSED_INPUT' '/Power Delivery/RAW_INPUT' VBUS +5VUSB '/PGM MUX, Headers/USB Programmer/VCCIO' P2_V32 P2_V40 P2_V56 --power-nets-widths 0.4 0.5 1.5 1.5 1.5 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signals.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND +5V '/I2C Outputs/Servo Controller/VMOT' '/Power Delivery/FUSED_INPUT' '/Power Delivery/RAW_INPUT' VBUS +5VUSB '/PGM MUX, Headers/USB Programmer/VCCIO' P2_V32 P2_V40 P2_V56 --power-nets-widths 0.4 0.5 1.5 1.5 1.5 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signals.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND +5V '/I2C Outputs/Servo Controller/VMOT' '/Power Delivery/FUSED_INPUT' '/Power Delivery/RAW_INPUT' VBUS +5VUSB '/PGM MUX, Headers/USB Programmer/VCCIO' P2_V32 P2_V40 P2_V56 --power-nets-widths 0.4 0.5 1.5 1.5 1.5 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signals.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND +5V '/I2C Outputs/Servo Controller/VMOT' '/Power Delivery/FUSED_INPUT' '/Power Delivery/RAW_INPUT' VBUS +5VUSB '/PGM MUX, Headers/USB Programmer/VCCIO' P2_V32 P2_V40 P2_V56 --power-nets-widths 0.4 0.5 1.5 1.5 1.5 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signals.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND +5V '/I2C Outputs/Servo Controller/VMOT' '/Power Delivery/FUSED_INPUT' '/Power Delivery/RAW_INPUT' VBUS +5VUSB '/PGM MUX, Headers/USB Programmer/VCCIO' P2_V32 P2_V40 P2_V56 --power-nets-widths 0.4 0.5 1.5 1.5 1.5 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/kevinbot_v3
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signals.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND +5V '/I2C Outputs/Servo Controller/VMOT' '/Power Delivery/FUSED_INPUT' '/Power Delivery/RAW_INPUT' VBUS +5VUSB '/PGM MUX, Headers/USB Programmer/VCCIO' P2_V32 P2_V40 P2_V56 --power-nets-widths 0.4 0.5 1.5 1.5 1.5 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
