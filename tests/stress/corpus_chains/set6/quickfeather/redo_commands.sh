#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set6/quickfeather.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.5 --via-drill 0.2 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' --clearance 0.09 --track-width 0.1 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.1 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u6.kicad_pcb step3_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpairs.kicad_pcb /S3_USB_D_P /S3_USB_D_N /USB.D_P /USB.D_N --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.2 --no-gnd-vias --layers F.Cu In1.Cu In2.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.09 --track-width 0.15 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +VBUS +VBAT --power-nets-widths 0.3 0.3 0.4 0.4 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5b_route_thin.kicad_pcb --nets '*' --clearance-ceiling 0.09 --track-width 0.1 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +VBUS +VBAT --power-nets-widths 0.3 0.3 0.4 0.4 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5b_route_thin.kicad_pcb step6_planefix.kicad_pcb --nets +3V3 GND --rip-existing-nets +3V3 GND --clearance-ceiling 0.09 --track-width 0.1 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 --power-nets-widths 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5b_route_thin.kicad_pcb step6_planefix.kicad_pcb --nets +3V3 GND --rip-existing-nets +3V3 GND --force-reroute --clearance-ceiling 0.09 --track-width 0.1 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 --power-nets-widths 0.3 0.3 --max-ripup 5 --no-bga-zone --overwrite=false
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5b_route_thin.kicad_pcb step6_planefix.kicad_pcb --nets +3V3 GND --rip-existing-nets +3V3 GND --force-reroute --clearance-ceiling 0.09 --track-width 0.1 --via-size 0.5 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 --power-nets-widths 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set6/quickfeather
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step5b_route_thin.kicad_pcb step6_planes_refinalize.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.5 --via-drill 0.2 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2
