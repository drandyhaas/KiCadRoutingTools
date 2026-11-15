#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/nudac.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.16 --zone-clearance 0.16 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.254 --track-width 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_U3.kicad_pcb --component U3 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_U3.kicad_pcb --component U3 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_U3.kicad_pcb --output step2b_fanout_U5.kicad_pcb --component U5 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_U5.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb step3_diff.kicad_pcb /Codec/MIC+ /Codec/MIC- USB_DP USB_DN --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.254 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/nudac
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.16 --track-width 0.25 --via-size 0.45 --via-drill 0.254 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 +5V VBUS /Codec/VDDA /MCU/AVDD /Codec/VDDC /Codec/VDDMIC /Codec/VDDB +1V8 VDDIO --power-nets-widths 0.4 0.4 0.4 0.5 0.3 0.3 0.3 0.3 0.3 0.3 0.3 --max-ripup 5
