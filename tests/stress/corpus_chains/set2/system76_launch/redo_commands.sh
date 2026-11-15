#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch
# [2026-08-19] removed dead command: '--timeout' is redo_stress_test.py's flag, never a routing-tool flag -- it exited rc=2 at record time too, and under 'set -e' it killed the whole replay at this line. Superseded by a clean command in this manifest.
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch
# [2026-08-19] removed dead command: '--timeout' is redo_stress_test.py's flag, never a routing-tool flag -- it exited rc=2 at record time too, and under 'set -e' it killed the whole replay at this line. Superseded by a clean command in this manifest.
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/system76_launch.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch/step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.16 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch/step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.16 --track-width 0.25 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND +3V6 --power-nets-widths 0.6 0.6
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch/step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.16 --track-width 0.25 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND +3V6 --power-nets-widths 0.6 0.6
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch/step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.16 --track-width 0.25 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND +3V6 --power-nets-widths 0.6 0.6
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch/step1_planes.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set2/system76_launch/step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.16 --track-width 0.25 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND +3V6 --power-nets-widths 0.6 0.6
