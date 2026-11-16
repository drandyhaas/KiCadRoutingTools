#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/arctyx_nano
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/arctyx_nano
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/arctyx_nano
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/arctyx_nano.kicad_pcb step1_planes.kicad_pcb --nets GND 3v3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --zone-clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/arctyx_nano
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U2 --nets '*' '!GND' '!3v3' --clearance 0.09 --width 0.1 --grid-step 0.05 --output step1b_fanout_u2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/arctyx_nano
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_fanout_u2.kicad_pcb --component U1 --nets '*' '!GND' '!3v3' --clearance 0.1 --width 0.1 --grid-step 0.05 --output step1b_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/arctyx_nano
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_fanout_u1.kicad_pcb --component U4 --nets '*' '!GND' '!3v3' --clearance 0.1 --width 0.1 --grid-step 0.05 --output step1b_fanout_u4.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/arctyx_nano
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanout_u4.kicad_pcb step1c_capopt.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/arctyx_nano
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/arctyx_nano
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1c_capopt.kicad_pcb step2a_diff.kicad_pcb --nets D+ D- --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/arctyx_nano
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/arctyx_nano
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step2_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --power-nets GND 3v3 1v1 1v2 VBUS 'Net-(U2-VREG_AVDD)' --power-nets-widths 0.3 0.3 0.25 0.25 0.3 0.2 --max-ripup 5
