#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/tinytapeout_qfn.kicad_pcb -o step1_fanout_u4.kicad_pcb -c U4 --nets '*' '!GND' '!VDDA' --clearance 0.09 --width 0.16 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_u4.kicad_pcb -o step2_fanout_u3.kicad_pcb -c U3 --nets '*' '!GND' '!VDDA' --clearance 0.09 --width 0.16 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u3.kicad_pcb step3_decap.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step3_decap.kicad_pcb step4_planes.kicad_pcb --nets GND VDDA --plane-layers In1.Cu In2.Cu --via-size 0.5 --via-drill 0.25 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --power-nets GND VDDA --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND VDDA 'VDD_{IO}' VCCD 'VDD_{REG}' 'VCC_{REG}' +3V3 +5V --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND VDDA 'VDD_{IO}' VCCD 'VDD_{REG}' 'VCC_{REG}' +3V3 +5V --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND VDDA 'VDD_{IO}' VCCD 'VDD_{REG}' 'VCC_{REG}' +3V3 +5V --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND VDDA 'VDD_{IO}' VCCD 'VDD_{REG}' 'VCC_{REG}' +3V3 +5V --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND VDDA 'VDD_{IO}' VCCD 'VDD_{REG}' 'VCC_{REG}' +3V3 +5V --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/tinytapeout_qfn
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step6_retry.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND VDDA 'VDD_{IO}' VCCD 'VDD_{REG}' 'VCC_{REG}' +3V3 +5V --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 0.3 0.3 --max-ripup 5
