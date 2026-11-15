#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/hackgdl_badge.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb step2_u8_fanout_TRY.kicad_pcb --component U8 --layers F.Cu B.Cu --nets '*' '!GND' --diff-pairs '/Cam/MCSI_*' /MCU/D+ /MCU/D- --diff-pair-gap 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --track-width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_u8_fanout_TRY.kicad_pcb --component U8 --layers F.Cu B.Cu --nets '*' '!GND' --diff-pairs '/Cam/MCSI_*' /MCU/D+ /MCU/D- --diff-pair-gap 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --track-width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_u8_fanout_TRY2.kicad_pcb --component U8 --layers F.Cu B.Cu --nets '*' '!GND' --diff-pairs '/Cam/MCSI_*' /MCU/D+ /MCU/D- --diff-pair-gap 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --track-width 0.1 --escape-method dogbone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_u8_fanout_TRY.kicad_pcb --output step3_u7_fanout.kicad_pcb --component U7 --nets '*' '!GND' --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.127
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_u7_fanout.kicad_pcb step4_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4_capclear.kicad_pcb step5_diffpairs.kicad_pcb '/Cam/MCSI_*' /MCU/D+ /MCU/D- --layers F.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diffpairs.kicad_pcb step6_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 /MCU/VDD_ARM VCC +1V8 +3.3V /Buck_Converter/VCC_DDR VBUS /MCU/VCC1V8_CODEC /MCU/VCC1V8_OTP 'Net-(U8A-RTC_AVDD3V3)' 'Net-(U8D-ETH_AVDD3V3)' /Buck_Converter/LX1 /Buck_Converter/LX2 /Buck_Converter/LX3 /Buck_Converter/LX4 --power-nets-widths 0.4 0.4 0.5 0.4 0.4 0.35 0.4 0.5 0.3 0.3 0.25 0.25 0.4 0.4 0.4 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/hackgdl_badge
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diffpairs.kicad_pcb step6b_route_thin.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.127 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 /MCU/VDD_ARM VCC +1V8 +3.3V /Buck_Converter/VCC_DDR VBUS /MCU/VCC1V8_CODEC /MCU/VCC1V8_OTP 'Net-(U8A-RTC_AVDD3V3)' 'Net-(U8D-ETH_AVDD3V3)' /Buck_Converter/LX1 /Buck_Converter/LX2 /Buck_Converter/LX3 /Buck_Converter/LX4 --power-nets-widths 0.4 0.4 0.5 0.4 0.4 0.35 0.4 0.5 0.3 0.3 0.25 0.25 0.4 0.4 0.4 0.4 --max-ripup 5
