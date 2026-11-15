#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set28/rein_r1.kicad_pcb --component U1 --output step1_fo_u1.kicad_pcb --width 0.1 --clearance 0.1 --nets '*' '!/GND' '!/3V3' --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fo_u1.kicad_pcb --component U3 --output step1_fo_u3.kicad_pcb --width 0.1 --clearance 0.1 --nets '*' '!/GND' '!/3V3' --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fo_u3.kicad_pcb --component U17 --output step1_fo_u17.kicad_pcb --width 0.1 --clearance 0.1 --nets '*' '!/GND' '!/3V3' --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fo_u17.kicad_pcb --component U18 --output step1_fo_u18.kicad_pcb --width 0.1 --clearance 0.1 --nets '*' '!/GND' '!/3V3' --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step1_fo_u18.kicad_pcb --output step2_planes.kicad_pcb --nets /GND /3V3 --plane-layers In2.Cu In1.Cu --via-size 0.5 --via-drill 0.2 --clearance 0.2 --hole-to-hole-clearance 0.25 --track-width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_planes.kicad_pcb --output step3_diff.kicad_pcb --nets '/USB_*_D[PM]' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1 -1 -1 1 --track-width 0.2 --diff-pair-gap 0.25 --clearance 0.2 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb --output step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1 -1 -1 1 --track-width 0.2 --clearance 0.2 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets /GND /3V3 /5V /VBUS /VBAT /VIN --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb --output step5_route_thin.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1 -1 -1 1 --track-width 0.127 --clearance 0.15 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets /GND /3V3 /5V /VBUS /VBAT /VIN --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/rein_r1
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route_thin.kicad_pcb --output step6_retry.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1 -1 -1 1 --track-width 0.1 --clearance 0.1 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets /GND /3V3 /5V /VBUS /VBAT /VIN --power-nets-widths 0.3 0.3 0.4 0.4 0.3 0.3 --max-ripup 5
