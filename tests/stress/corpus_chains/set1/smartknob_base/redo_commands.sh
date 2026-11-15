#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set1/smartknob_base.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base/step1_planes.kicad_pcb --nets GND GNDA --plane-layers B.Cu B.Cu --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --track-width 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base/step1_planes.kicad_pcb --output /Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base/step2_qfn_fanout.kicad_pcb --component U5 --clearance 0.1 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py /Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base/step2_qfn_fanout.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base/step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py /Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base/step3_capclear.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base/step4_diffpairs.kicad_pcb --nets /STRAIN_S+ /STRAIN_S- /USB_D+ /USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base/step4_diffpairs.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set1/smartknob_base/step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets GND GNDA +3V3 +5V 'Net-(J1-Pad1)' 'Net-(J1-Pad2)' 'Net-(J1-Pad3)' --power-nets-widths 0.2 0.2 0.4 0.4 0.5 0.5 0.5
