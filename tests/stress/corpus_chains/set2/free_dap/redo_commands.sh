#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/free_dap
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/free_dap
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/free_dap
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/free_dap.kicad_pcb step1_planes.kicad_pcb --nets GND GND --plane-layers F.Cu B.Cu --via-size 0.45 --via-drill 0.3302 --clearance-ceiling 0.1778 --hole-to-hole-clearance 0.254 --power-nets +3V3 VBUS --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/free_dap
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout_ic2.kicad_pcb --component IC2 --nets '!GND' '!+3V3' '!VBUS' --width 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/free_dap
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_ic2.kicad_pcb --component IC2 --nets '!GND' '!+3V3' '!VBUS' --width 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/free_dap
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_ic2.kicad_pcb step2b_capopt.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/free_dap
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/free_dap
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_capopt.kicad_pcb --output step3_diffpair.kicad_pcb --nets /USB_DP /USB_DM --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3302 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/free_dap
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/free_dap
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpair.kicad_pcb --output step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1778 --track-width 0.1905 --via-size 0.45 --via-drill 0.3302 --hole-to-hole-clearance 0.254 --power-nets +3V3 VBUS --power-nets-widths 0.3 0.3
