#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/zynthian_v5_control
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/copy_board.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set13/zynthian_v5_control.kicad_pcb board0.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/zynthian_v5_control
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py board0.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.19 --track-width 0.15 --output board1a.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/zynthian_v5_control
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py board0.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.19 --width 0.15 --output board1a.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/zynthian_v5_control
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py board1a.kicad_pcb --component U2 --nets '*' '!GND' --clearance 0.19 --width 0.15 --output board1b.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/zynthian_v5_control
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py board1b.kicad_pcb board1c.kicad_pcb --clearance 0.19
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/zynthian_v5_control
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board1c.kicad_pcb board2.kicad_pcb --nets '*' '!GND' --power-nets 5v 3.3v --power-nets-widths 0.4 0.3 --clearance-ceiling 0.15 --track-width 0.15 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --layer-costs 1.0 1.5 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/zynthian_v5_control
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board1c.kicad_pcb board2.kicad_pcb --nets '*' '!GND' --power-nets 5v 3.3v --power-nets-widths 0.4 0.3 --clearance-ceiling 0.15 --track-width 0.15 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --layer-costs 1.0 1.5 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/zynthian_v5_control
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py board2.kicad_pcb board3.kicad_pcb --nets GND GND --plane-layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/zynthian_v5_control
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py board3.kicad_pcb board4_final.kicad_pcb --nets GND --power-nets GND --power-nets-widths 0.4 --clearance-ceiling 0.15 --track-width 0.15 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --layer-costs 1.0 1.5 --no-bga-zone --max-ripup 5
