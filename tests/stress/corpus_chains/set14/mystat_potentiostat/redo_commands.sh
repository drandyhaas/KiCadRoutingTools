#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mystat_potentiostat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mystat_potentiostat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/mystat_potentiostat.kicad_pcb step1_planes.kicad_pcb --nets GND GND --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.254 --track-width 0.254 --clearance-ceiling 0.254 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mystat_potentiostat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mystat_potentiostat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_U2.kicad_pcb --component U2 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' --clearance 0.254 --track-width 0.254 --via-size 0.45 --via-drill 0.254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mystat_potentiostat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mystat_potentiostat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_U2.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.254
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mystat_potentiostat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/mystat_potentiostat
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_capclear.kicad_pcb step3_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.254 --track-width 0.254 --via-size 0.45 --via-drill 0.254 --hole-to-hole-clearance 0.2 --power-nets GND +13.4V -13.4V +5V +15V -15V --power-nets-widths 0.5 0.4 0.4 0.4 0.4 0.4 --stats
