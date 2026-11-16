#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/mdfpga_hub75
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/mdfpga_hub75
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/mdfpga_hub75
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py mdfpga_hub75.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In2.Cu In1.Cu --clearance-ceiling 0.1905 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/mdfpga_hub75
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout.kicad_pcb --component U3 --nets '*' '!GND' '!+3V3' --clearance 0.1905 --width 0.1905
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/mdfpga_hub75
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component U3 --nets '*' '!GND' '!+3V3' --clearance 0.1905 --width 0.1905
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/mdfpga_hub75
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step3_declutter.kicad_pcb --clearance 0.1905
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/mdfpga_hub75
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/mdfpga_hub75
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/mdfpga_hub75
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_declutter.kicad_pcb step4_route.kicad_pcb '*' --clearance-ceiling 0.1905 --track-width 0.1905 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3V3 +1V2 +5V /VBAT --power-nets-widths 0.254 0.254 0.254 1.27 0.508
