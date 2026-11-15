#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
# REBUILT from redo_commands.timings.jsonl after a relaunch clobbered the
# original (see run_board.sh archive-on-relaunch fix). Chain walked backward
# from the graded board (step2_route.kicad_pcb); commands are byte-faithful re-quotes.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/eez_dib_b3c
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set16/eez_dib_b3c.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.5 --via-drill 0.3 --clearance-ceiling 0.2032 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/eez_dib_b3c
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2032 --track-width 0.254 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V +12V +3V3 --power-nets-widths 0.6 0.6 0.6 0.5
