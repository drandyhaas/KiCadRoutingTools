#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/uhk_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/uhk_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set8/uhk_trackball.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/uhk_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/uhk_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.1 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/uhk_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step3_declear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/uhk_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/uhk_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_declear.kicad_pcb step4a_route_qfn.kicad_pcb --nets /SW1 /SHTDWN /MISO /MOSI /SCK /NCS /TEST_LED /SCL /SDA /MOTION /SW2 /SWDCLK /RESET /SWDIO --clearance-ceiling 0.15 --track-width 0.15 --grid-step 0.05 --via-size 0.7112 --via-drill 0.381 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/uhk_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4a_route_qfn.kicad_pcb step4b_route_all.kicad_pcb --nets '*' --clearance-ceiling 0.2032 --track-width 0.254 --grid-step 0.05 --via-size 0.7112 --via-drill 0.381 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V /ADNS_AVDD /ADNS_VDD /VCC_BOTTOM --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/uhk_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4b_route_all.kicad_pcb step5_fix_gnd.kicad_pcb --nets GND --rip-existing-nets GND --force-reroute --clearance-ceiling 0.2032 --track-width 0.254 --grid-step 0.05 --via-size 0.7112 --via-drill 0.381 --hole-to-hole-clearance 0.2 --power-nets GND --power-nets-widths 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/uhk_trackball
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step4b_route_all.kicad_pcb step5_replane_gnd.kicad_pcb --nets GND --plane-layers B.Cu
