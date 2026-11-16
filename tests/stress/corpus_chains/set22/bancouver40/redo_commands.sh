#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/private/tmp/wt_baseline_0809
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/bancouver40.kicad_pcb board_step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py board_step1_planes.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.1 --grid-step 0.05 --output board_step1b_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py board_step1b_fanout.kicad_pcb board_step1c_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py board_step1c_capclear.kicad_pcb board_step2a_diff.kicad_pcb --nets D+ D- D_res+ D_res- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py board_step2a_diff.kicad_pcb board_step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.127 --via-size 0.45 --via-drill 0.3 --power-nets GND +5V VBUS --power-nets-widths 0.3 0.4 0.4 --layers F.Cu B.Cu --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py board_step2_route.kicad_pcb board_step2_fix_col9.kicad_pcb --nets col9 --force-reroute --clearance-ceiling 0.2 --track-width 0.127 --via-size 0.45 --via-drill 0.3 --layers F.Cu B.Cu --grid-step 0.05 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py board_step2a_diff.kicad_pcb board_step2_route_balanced.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.127 --via-size 0.45 --via-drill 0.3 --power-nets GND +5V VBUS --power-nets-widths 0.3 0.4 0.4 --layers F.Cu B.Cu --layer-costs 1.0 1.5 --grid-step 0.05 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/bancouver40
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py board_step2a_diff.kicad_pcb board_step2_route_balanced2.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.127 --via-size 0.45 --via-drill 0.3 --power-nets GND +5V VBUS --power-nets-widths 0.3 0.4 0.4 --layers F.Cu B.Cu --layer-costs 1.0 1.0 --grid-step 0.05 --no-bga-zone --max-ripup 5
