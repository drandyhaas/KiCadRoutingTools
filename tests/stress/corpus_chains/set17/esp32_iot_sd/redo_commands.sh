#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set17/esp32_iot_sd.kicad_pcb step1_planes.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_u1.kicad_pcb --component U1 --clearance 0.15 --width 0.15 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --clearance 0.15 --width 0.15 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u1.kicad_pcb --output step2b_fanout_u3.kicad_pcb --component U3 --clearance 0.15 --width 0.15 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u3.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb --output step3_diffpairs.kicad_pcb --nets USB_DP USB_DN /ESP32_SDCard_FlashMemory_USBModule/USB_D+ /ESP32_SDCard_FlashMemory_USBModule/USB_D- --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/esp32_iot_sd
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb --output step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +5V VBUS '/Power_and USB_Module/VBUS' '/Power_and USB_Module/VBAT' --power-nets-widths 0.5 0.5 0.25 0.6 0.6 0.6
