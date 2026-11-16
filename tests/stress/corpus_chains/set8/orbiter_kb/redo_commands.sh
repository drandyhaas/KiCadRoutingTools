#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set8/orbiter_kb.kicad_pcb --component MCU1 --clearance 0.1 --width 0.15 --grid-step 0.1 --output step1_mcu1_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_mcu1_fanout.kicad_pcb --component MCU2 --clearance 0.1 --width 0.15 --grid-step 0.1 --output step2_mcu2_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_mcu2_fanout.kicad_pcb --nets /USB_DP /USB_DN /USB_CONN_DP /USB_CONN_DN --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --no-gnd-vias --output step3_diffpairs.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step3_diffpairs.kicad_pcb step4_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --power-nets /TPS_VOUT /TPS_EN --power-nets-widths 0.4 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets /TPS_VOUT /TPS_EN GND --power-nets-widths 0.4 0.3 0.3 --max-ripup 5 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets /TPS_VOUT /TPS_EN GND --power-nets-widths 0.4 0.3 0.3 --max-ripup 5 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/orbiter_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5b_route_thin.kicad_pcb --nets '*' --clearance-ceiling 0.12 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets /TPS_VOUT /TPS_EN GND --power-nets-widths 0.4 0.3 0.3 --max-ripup 5 --grid-step 0.05
