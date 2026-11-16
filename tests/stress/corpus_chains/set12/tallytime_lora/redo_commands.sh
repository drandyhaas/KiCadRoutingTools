#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/tallytime_lora.kicad_pcb -c U2 --nets '*' '!GND' '!+3V3' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --grid-step 0.05 --output step1_fanout_U2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_fanout_U2.kicad_pcb step2_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_capclear.kicad_pcb step3_diffpairs.kicad_pcb '*USB*D*' '*USBESD*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2395 --diff-pair-gap 0.15 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --no-gnd-vias --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step3_diffpairs.kicad_pcb step4_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --track-width 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/tallytime_lora
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --power-nets GND +3V3 +5V VBUS +BATT +1V1 +VIN --power-nets-widths 0.4 0.4 0.3 0.4 0.4 0.3 0.4
