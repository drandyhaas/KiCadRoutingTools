#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set18/moteus_c1.kicad_pcb step1_planes.kicad_pcb --nets GND V+ GNDPWR --plane-layers In1.Cu In2.Cu In2.Cu --clearance-ceiling 0.127
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U3 --nets '*' '!GND' --clearance 0.127 --width 0.1524 --output step1b_u3_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U3 --nets '*' '!GND' --clearance 0.1 --width 0.1 --output step1b_u3_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U3 --nets '*' '!GND' --clearance 0.127 --width 0.1524 --output step1b_u3_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_u3_fanout.kicad_pcb --component U4 --nets '*' '!GNDPWR' '!V+' --clearance 0.127 --width 0.1524 --output step1b_u4_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_u4_fanout.kicad_pcb step1c_capopt.kicad_pcb --clearance 0.127
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1c_capopt.kicad_pcb step2a_diff_cursense.kicad_pcb --nets '/H Bridge A/SXX_*' '/H Bridge B/SXX_*' '/H Bridge C/SXX_*' --track-width 0.145 --diff-pair-gap 0.145 --clearance-ceiling 0.127 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2a_diff_cursense.kicad_pcb step2a_diff_can.kicad_pcb --nets '/CAN/CAN_*' --track-width 0.1524 --diff-pair-gap 0.127 --clearance-ceiling 0.127 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff_can.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.1524 --via-size 0.45 --via-drill 0.254 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND GNDPWR V+ +3V +5V '/H Bridge A/OUTX' '/H Bridge B/OUTX' '/H Bridge C/OUTX' '/Gate Driver/GHA' '/Gate Driver/GHB' '/Gate Driver/GHC' '/Gate Driver/GLA' '/Gate Driver/GLB' '/Gate Driver/GLC' --power-nets-widths 0.3 0.3 0.3 0.25 0.3 0.4 0.4 0.4 0.384 0.384 0.384 0.384 0.384 0.384 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff_can.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.1524 --via-size 0.45 --via-drill 0.254 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND GNDPWR V+ +3V +5V '/H Bridge A/OUTX' '/H Bridge B/OUTX' '/H Bridge C/OUTX' '/Gate Driver/GHA' '/Gate Driver/GHB' '/Gate Driver/GHC' '/Gate Driver/GLA' '/Gate Driver/GLB' '/Gate Driver/GLC' --power-nets-widths 0.3 0.3 0.3 0.25 0.3 0.4 0.4 0.4 0.384 0.384 0.384 0.384 0.384 0.384 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff_can.kicad_pcb step2_route_v2.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.127 --via-size 0.45 --via-drill 0.254 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5 --power-nets GND GNDPWR V+ +3V +5V '/H Bridge A/OUTX' '/H Bridge B/OUTX' '/H Bridge C/OUTX' '/Gate Driver/GHA' '/Gate Driver/GHB' '/Gate Driver/GHC' '/Gate Driver/GLA' '/Gate Driver/GLB' '/Gate Driver/GLC' --power-nets-widths 0.3 0.3 0.3 0.25 0.3 0.4 0.4 0.4 0.384 0.384 0.384 0.384 0.384 0.384 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set18/moteus_c1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step2c_sxx_retry.kicad_pcb --nets '/H Bridge A/SXX_*' '/H Bridge B/SXX_*' '/H Bridge C/SXX_*' --clearance-ceiling 0.09 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0
