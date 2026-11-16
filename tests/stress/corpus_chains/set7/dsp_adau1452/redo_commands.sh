#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/dsp_adau1452
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set7/dsp_adau1452.kicad_pcb step1_planes.kicad_pcb --nets GNDD VAA --plane-layers In1.Cu In2.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/dsp_adau1452
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2_fanout.kicad_pcb --component U201 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/dsp_adau1452
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step3_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set7/dsp_adau1452
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_clearance.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GNDD VAA /ADAU1452/DVDD /ADAU1452/IOVDD VCC VDD '/Power Supply/2V1' /ADAU1452/PVDD /ADAU1452/AVDD 'Net-(C402-Pad1)' --power-nets-widths 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 --max-ripup 5
