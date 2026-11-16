#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/rp2350_dev
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/rp2350_dev
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/rp2350_dev.kicad_pcb --component U1 --output step1_fanout_u1.kicad_pcb --nets '*' '!GND' --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/rp2350_dev
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_u1.kicad_pcb --component U3 --output step2_fanout_u3.kicad_pcb --nets '*' '!GND' --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/rp2350_dev
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u3.kicad_pcb step3_fanout_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/rp2350_dev
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/rp2350_dev
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_fanout_clearance.kicad_pcb step4_diffpairs.kicad_pcb '*USB_D*' --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --no-gnd-vias --layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/rp2350_dev
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/rp2350_dev
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step4_diffpairs.kicad_pcb step5_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --power-nets +3.3V VBUS +1V1 /VREG_AVDD --power-nets-widths 0.5 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/rp2350_dev
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/rp2350_dev
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND +3.3V VBUS +1V1 /VREG_AVDD --power-nets-widths 0.5 0.5 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/rp2350_dev
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --grid-step 0.05 --power-nets GND +3.3V VBUS +1V1 /VREG_AVDD --power-nets-widths 0.5 0.5 0.4 0.3 0.3
