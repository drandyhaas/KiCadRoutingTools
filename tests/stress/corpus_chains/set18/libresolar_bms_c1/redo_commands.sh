#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/libresolar_bms_c1.kicad_pcb step1_planes.kicad_pcb --nets GND GND +3V3 --plane-layers In1.Cu In2.Cu In2.Cu --power-nets GND +3V3 --power-nets-widths 0.4 0.4 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U4 --output step2_fanout_u4.kicad_pcb --nets '*' '!GND' '!+3V3' --clearance 0.2 --width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u4.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_capclear.kicad_pcb step3a_diff_usb.kicad_pcb '/ESP32-C3 MCU/USB_D+' '/ESP32-C3 MCU/USB_D-' --impedance 90 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.45 --via-drill 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3a_diff_usb.kicad_pcb step3b_diff_sense.kicad_pcb /BQ76952/SRP /BQ76952/SRN /BQ76952/SHUNT_P /BQ76952/SHUNT_N --diff-pair-gap 0.2 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3b_diff_sense.kicad_pcb step4_hv.kicad_pcb --nets /BAT+ /BAT- /PACK+ /PACK- /CHG+ '/Power Supply/VIN' --clearance-ceiling 0.3 --track-width 0.3 --power-nets /BAT+ /BAT- /PACK+ /PACK- /CHG+ '/Power Supply/VIN' --power-nets-widths 1.5 1.5 1.5 1.5 1.5 1.0 --via-size 0.8 --via-drill 0.4 --hole-to-hole-clearance 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/libresolar_bms_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_hv.kicad_pcb step5_main.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --power-nets GND +3V3 --power-nets-widths 0.4 0.4 --hole-to-hole-clearance 0.2 --max-ripup 5
