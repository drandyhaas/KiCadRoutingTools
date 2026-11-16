#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/eit_mux.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.15 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
# [2026-08-19] removed dead command: '--timeout' is redo_stress_test.py's flag, never a routing-tool flag -- it exited rc=2 at record time too, and under 'set -e' it killed the whole replay at this line. Superseded by a clean command in this manifest.
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2_fanout_u3.kicad_pcb -c U3 --nets '*' '!GND' --width 0.15 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_fanout_u3.kicad_pcb -o step2b_fanout_u5.kicad_pcb -c U5 --nets '*' '!GND' --width 0.15 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_u5.kicad_pcb -o step2c_fanout_u6.kicad_pcb -c U6 --nets '*' '!GND' --width 0.15 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fanout_u6.kicad_pcb -o step2d_fanout_u7.kicad_pcb -c U7 --nets '*' '!GND' --width 0.15 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2d_fanout_u7.kicad_pcb -o step2e_fanout_u8.kicad_pcb -c U8 --nets '*' '!GND' --width 0.15 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2e_fanout_u8.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diff.kicad_pcb --nets '*USB*' --track-width 0.15 --diff-pair-gap 0.15 --clearance-ceiling 0.15 --via-size 0.6 --via-drill 0.35 --no-gnd-vias --layers F.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.6 --via-drill 0.35 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.3 --power-nets GND +3V3 -3V3 +1V1 VBUS --power-nets-widths 0.3 0.5 0.4 0.3 0.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.6 --via-drill 0.35 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.3 --power-nets GND +3V3 -3V3 +1V1 VBUS --power-nets-widths 0.3 0.5 0.4 0.3 0.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.6 --via-drill 0.35 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.3 --power-nets GND +3V3 -3V3 +1V1 VBUS --power-nets-widths 0.3 0.5 0.4 0.3 0.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.6 --via-drill 0.35 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.3 --power-nets GND +3V3 -3V3 +1V1 VBUS --power-nets-widths 0.3 0.5 0.4 0.3 0.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.6 --via-drill 0.35 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.3 --power-nets GND +3V3 -3V3 +1V1 VBUS --power-nets-widths 0.3 0.5 0.4 0.3 0.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/eit_mux
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step5b_retry.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.1 --via-size 0.6 --via-drill 0.35 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.3 --power-nets GND +3V3 -3V3 +1V1 VBUS --power-nets-widths 0.3 0.5 0.4 0.3 0.5 --max-ripup 5
