#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
# REBUILT from redo_commands.timings.jsonl after a relaunch clobbered the
# original (see run_board.sh archive-on-relaunch fix). Chain walked backward
# from the graded board (step2c_final.kicad_pcb); commands are byte-faithful re-quotes.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/abn6502
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/abn6502.kicad_pcb step1_planes.kicad_pcb --nets GND GND --plane-layers F.Cu B.Cu --clearance-ceiling 0.13 --zone-clearance 0.13
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/abn6502
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2a_power.kicad_pcb --nets GND +5V +3V3 --power-nets GND +5V +3V3 --power-nets-widths 0.3 0.3 0.25 --clearance-ceiling 0.13 --track-width 0.1 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --layers F.Cu B.Cu --max-ripup 5
# recovered from transcript.jsonl (step never recorded: backgrounded) (recorded to no manifest)
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/abn6502
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_power.kicad_pcb step2b_signal.kicad_pcb --nets '*' --power-nets GND +5V +3V3 --power-nets-widths 0.3 0.3 0.25 --clearance-ceiling 0.13 --track-width 0.1 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --layers F.Cu B.Cu --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/abn6502
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2b_signal.kicad_pcb step2c_final.kicad_pcb --nets '*' '!C5' '!C2' '!Ø1B' '!D4' '!PB6' '!~{RW}' '!~{IO_SEL}' '!~{RES}' --power-nets GND +5V +3V3 --power-nets-widths 0.3 0.3 0.25 --clearance-ceiling 0.13 --track-width 0.1 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --layers F.Cu B.Cu --max-ripup 5
