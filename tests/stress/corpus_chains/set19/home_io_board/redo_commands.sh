#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/home_io_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set19/home_io_board.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/home_io_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.7 --track-width 0.54 --via-size 0.5 --via-drill 0.3 --power-nets GND +12V +3.3V +5V --power-nets-widths 0.6 0.8 0.6 0.6 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/home_io_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.7 --track-width 0.54 --via-size 0.5 --via-drill 0.3 --power-nets GND +12V +3.3V +5V --power-nets-widths 0.6 0.8 0.6 0.6 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/home_io_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.7 --track-width 0.54 --via-size 0.5 --via-drill 0.3 --power-nets GND +12V +3.3V +5V --power-nets-widths 0.6 0.8 0.6 0.6 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/home_io_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/home_io_board
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step2b_gnd_retry.kicad_pcb --nets GND --rip-existing-nets +3.3V +12V +5V 'Net-(Q*' 'REL*' --clearance-ceiling 0.7 --track-width 0.54 --via-size 0.5 --via-drill 0.3 --power-nets GND +12V +3.3V +5V --power-nets-widths 0.6 0.8 0.6 0.6 --no-bga-zone --grid-step 0.05 --max-ripup 5
