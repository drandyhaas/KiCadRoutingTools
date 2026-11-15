#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/usb_pd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/usb_pd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/usb_pd.kicad_pcb --component U101 --clearance 0.09 --grid-step 0.05 --output step1_fanout_u101.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/usb_pd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_fanout_u101.kicad_pcb step2_place_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/usb_pd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/usb_pd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/usb_pd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step2_place_clearance.kicad_pcb step3_planes.kicad_pcb --nets GND VBUS --plane-layers In1.Cu In2.Cu --via-size 0.508 --via-drill 0.254 --track-width 0.13 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND VBUS --power-nets-widths 0.3 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/usb_pd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/usb_pd
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_planes.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.13 --via-size 0.508 --via-drill 0.254 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND VBUS --power-nets-widths 0.3 0.3 --stats
