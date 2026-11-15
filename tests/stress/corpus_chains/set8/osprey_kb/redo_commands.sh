#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set8/osprey_kb.kicad_pcb --component U1 --nets '*' '!GND' --escape-method underpad --allow-via-in-pad --clearance 0.1 --output step1_u1fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set8/osprey_kb.kicad_pcb --component U1 --nets '*' '!GND' --escape-method underpad --allow-via-in-pad --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.1 --output step1_u1fanout_b.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_u1fanout_b.kicad_pcb --component U2 --nets '*' '!GND' --clearance 0.1 --output step1_u2fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_u2fanout.kicad_pcb --component U3 --nets '*' '!GND' --clearance 0.1 --output step1_u3fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_u3fanout.kicad_pcb step1c_capopt.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1c_capopt.kicad_pcb step2a_diff.kicad_pcb --nets USB-Di USB-Do --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diff.kicad_pcb step2_route.kicad_pcb --nets '*' '!GND' '!Earth' --layers F.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --power-nets +5V VDD VDDH VBUS GNDPWR --power-nets-widths 0.3 0.3 0.3 0.3 0.3 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step2_route_reset_retry.kicad_pcb --nets RESET --layers F.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --grid-step 0.05 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step2_route_reset_retry.kicad_pcb step3_planes.kicad_pcb --nets GND GND Earth Earth --plane-layers F.Cu B.Cu F.Cu B.Cu --clearance-ceiling 0.1 --zone-clearance 0.1 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_planes.kicad_pcb final_board.kicad_pcb --nets '*' --layers F.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND Earth +5V VDD VDDH VBUS GNDPWR --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.3 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py final_board.kicad_pcb final_board_sda_retry.kicad_pcb --nets SDA --layers F.Cu B.Cu --track-width 0.08 --clearance-ceiling 0.08 --hole-to-hole-clearance 0.2 --via-size 0.25 --via-drill 0.15 --grid-step 0.025 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/osprey_kb
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py final_board_sda_retry.kicad_pcb final_board_gnd_retry.kicad_pcb --nets GND --layers F.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --power-nets GND --power-nets-widths 0.3 --no-bga-zone --max-ripup 5
