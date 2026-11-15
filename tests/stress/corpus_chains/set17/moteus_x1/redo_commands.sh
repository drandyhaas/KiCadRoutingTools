#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set17/moteus_x1.kicad_pcb board_step1_planes.kicad_pcb --nets GND GNDPWR V+ +3V --plane-layers In1.Cu In4.Cu In2.Cu In3.Cu --via-size 0.45 --via-drill 0.254 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py board_step1_planes.kicad_pcb --component U4 --nets '*' '!GND' '!+3V' --clearance 0.1524 --grid-step 0.05 --output board_step1b_u4.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py board_step1b_u4.kicad_pcb --component U5 --nets '*' '!GND' '!GNDPWR' '!V+' '!+3V' --clearance 0.1524 --grid-step 0.05 --output board_step1b_u5.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py board_step1b_u5.kicad_pcb --component U7 --nets '*' '!GND' '!+3V' --clearance 0.1524 --grid-step 0.05 --output board_step1b_u7.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py board_step1b_u7.kicad_pcb board_step1c.kicad_pcb --clearance 0.1524
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py board_step1c.kicad_pcb board_step2a_diff.kicad_pcb --nets /CAN/CAN_P /CAN/CAN_N '/H Bridge A/SXX_P' '/H Bridge A/SXX_N' '/H Bridge B/SXX_P' '/H Bridge B/SXX_N' '/H Bridge C/SXX_P' '/H Bridge C/SXX_N' /rs422/RS422_RX_P /rs422/RS422_RX_N /rs422/RS422_TX_P /rs422/RS422_TX_N --track-width 0.1524 --diff-pair-gap 0.1524 --via-size 0.45 --via-drill 0.254 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.5 3.0 1.0 --grid-step 0.05 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board_step2a_diff.kicad_pcb board_step3_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND GNDPWR V+ +3V +12V '/Gate Driver/GHA' '/Gate Driver/GHB' '/Gate Driver/GHC' '/Gate Driver/GLA' '/Gate Driver/GLB' '/Gate Driver/GLC' '/H Bridge A/OUTX' '/H Bridge B/OUTX' '/H Bridge C/OUTX' --power-nets-widths 0.5 0.5 1.0 0.4 0.3 0.384 0.384 0.384 0.384 0.384 0.384 0.5 0.5 0.5 --via-size 0.45 --via-drill 0.254 --grid-step 0.05 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.5 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board_step2a_diff.kicad_pcb board_step3_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND GNDPWR V+ +3V +12V '/Gate Driver/GHA' '/Gate Driver/GHB' '/Gate Driver/GHC' '/Gate Driver/GLA' '/Gate Driver/GLB' '/Gate Driver/GLC' '/H Bridge A/OUTX' '/H Bridge B/OUTX' '/H Bridge C/OUTX' --power-nets-widths 0.5 0.5 1.0 0.4 0.3 0.384 0.384 0.384 0.384 0.384 0.384 0.5 0.5 0.5 --via-size 0.45 --via-drill 0.254 --grid-step 0.05 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.5 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board_step3_route.kicad_pcb board_step3_retry.kicad_pcb --nets '/H Bridge B/SXX_P' --rip-existing-nets '/Gate Driver/GLB' '/H Bridge B/SXX_N' '/H Bridge B/OUTX' --no-bga-zone --max-ripup 5 --power-nets '/Gate Driver/GLB' '/H Bridge B/OUTX' --power-nets-widths 0.384 0.5 --via-size 0.45 --via-drill 0.254 --grid-step 0.05 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.5 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board_step3_route.kicad_pcb board_step3_retry2.kicad_pcb --nets '/H Bridge B/SXX_P' --rip-existing-nets '/Gate Driver/GLB' '/H Bridge B/SXX_N' '/H Bridge B/OUTX' /aux1/AUX1_A '/H Bridge B/SPX_Q' --no-bga-zone --max-ripup 5 --clearance-ceiling 0.09 --track-width 0.0762 --power-nets '/Gate Driver/GLB' '/H Bridge B/OUTX' --power-nets-widths 0.384 0.5 --via-size 0.45 --via-drill 0.254 --grid-step 0.025 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.5 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board_step3_route.kicad_pcb board_step3_retry2.kicad_pcb --nets '/H Bridge B/SXX_P' --rip-existing-nets '/Gate Driver/GLB' '/H Bridge B/SXX_N' '/H Bridge B/OUTX' /aux1/AUX1_A '/H Bridge B/SPX_Q' --no-bga-zone --max-ripup 5 --clearance-ceiling 0.09 --track-width 0.0762 --power-nets '/Gate Driver/GLB' '/H Bridge B/OUTX' --power-nets-widths 0.384 0.5 --via-size 0.45 --via-drill 0.254 --grid-step 0.025 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.5 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/moteus_x1
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/copy_board.py board_step3_retry2.kicad_pcb final_board.kicad_pcb
