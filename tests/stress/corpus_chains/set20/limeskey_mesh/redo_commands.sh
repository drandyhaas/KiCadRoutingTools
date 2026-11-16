#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/limeskey_mesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set20/limeskey_mesh.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.15 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/limeskey_mesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2b_rf.kicad_pcb --nets GPS_ANT /LORA/LORA_ANT /GNSS/AFT_GPS_ANT /LORA/PRE_LORA_ANT --layers F.Cu --impedance 50 --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/limeskey_mesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_rf.kicad_pcb step3_signal.kicad_pcb --nets '*' '!GPS_ANT' '!/LORA/LORA_ANT' '!/GNSS/AFT_GPS_ANT' '!/LORA/PRE_LORA_ANT' --clearance-ceiling 0.15 --track-width 0.3 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V VBUS -BATT +BATT /GNSS/VCC_RF --power-nets-widths 0.3 0.3 0.4 0.5 0.5 0.5 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/limeskey_mesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_rf.kicad_pcb step3_signal.kicad_pcb --nets '*' '!GPS_ANT' '!/LORA/LORA_ANT' '!/GNSS/AFT_GPS_ANT' '!/LORA/PRE_LORA_ANT' --clearance-ceiling 0.15 --track-width 0.3 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V VBUS '*BATT' /GNSS/VCC_RF --power-nets-widths 0.3 0.3 0.4 0.5 0.5 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/limeskey_mesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/limeskey_mesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_rf.kicad_pcb step3b_signal_margin.kicad_pcb --nets '*' '!GPS_ANT' '!/LORA/LORA_ANT' '!/GNSS/AFT_GPS_ANT' '!/LORA/PRE_LORA_ANT' --clearance-ceiling 0.15 --track-width 0.3 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --routing-clearance-margin 1.3 --power-nets GND +3V3 +5V VBUS '*BATT' /GNSS/VCC_RF --power-nets-widths 0.3 0.3 0.4 0.5 0.5 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/limeskey_mesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_rf.kicad_pcb step3c_signal_outeronly.kicad_pcb --nets '*' '!GPS_ANT' '!/LORA/LORA_ANT' '!/GNSS/AFT_GPS_ANT' '!/LORA/PRE_LORA_ANT' --layers F.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.3 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V VBUS '*BATT' /GNSS/VCC_RF --power-nets-widths 0.3 0.3 0.4 0.5 0.5 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/limeskey_mesh
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_rf.kicad_pcb step3d_signal_clr02.kicad_pcb --nets '*' '!GPS_ANT' '!/LORA/LORA_ANT' '!/GNSS/AFT_GPS_ANT' '!/LORA/PRE_LORA_ANT' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.3 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V VBUS '*BATT' /GNSS/VCC_RF --power-nets-widths 0.3 0.3 0.4 0.5 0.5 0.3 --max-ripup 5
