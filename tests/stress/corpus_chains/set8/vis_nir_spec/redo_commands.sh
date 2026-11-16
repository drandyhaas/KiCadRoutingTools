#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/vis_nir_spec
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set8/vis_nir_spec.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/vis_nir_spec
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/vis_nir_spec
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U3 --nets '*' '!GND' --width 0.15 --clearance 0.1 --grid-step 0.05 --output step2_fanout_u3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/vis_nir_spec
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u3.kicad_pcb step3_capopt.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/vis_nir_spec
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capopt.kicad_pcb step4_diff.kicad_pcb --nets USB_D+ USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/vis_nir_spec
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/vis_nir_spec
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --power-nets GND +3.3V +VBAT +5V VBUS +1V1 --power-nets-widths 0.3 0.4 0.4 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/vis_nir_spec
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step6_fix33v.kicad_pcb --nets +3.3V --force-reroute --no-bga-zone --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --power-nets GND +3.3V +VBAT +5V VBUS +1V1 --power-nets-widths 0.3 0.4 0.4 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set8/vis_nir_spec
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_fix33v.kicad_pcb step7_fix33v2.kicad_pcb --nets +3.3V GND --force-reroute --no-bga-zone --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --via-cost 60 --power-nets GND +3.3V +VBAT +5V VBUS +1V1 --power-nets-widths 0.3 0.4 0.4 0.4 0.4 0.4
