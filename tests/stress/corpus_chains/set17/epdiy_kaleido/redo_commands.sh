#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set17/epdiy_kaleido.kicad_pcb step1_planes.kicad_pcb --nets GND GND 3.3V /VNEG_IN --plane-layers In1.Cu B.Cu In2.Cu In2.Cu --clearance-ceiling 0.144 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U3 --nets '*' '!GND' '!3.3V' '!/VNEG_IN' --clearance 0.144 --width 0.2 --grid-step 0.1 --output step1b_fanout_u3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U3 --nets '*' '!GND' '!3.3V' '!/VNEG_IN' --clearance 0.09 --width 0.15 --grid-step 0.05 --output step1b_fanout_u3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanout_u3.kicad_pcb step1c_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1c_clearance.kicad_pcb step2a_diffpairs.kicad_pcb --nets /ESP_USB+ /ESP_USB- /USB_D+ /USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.144 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND 3.3V /VNEG_IN VIN VBUS /+22V /-15V /-20V /15V EPD_VDD --power-nets-widths 0.3 0.3 0.3 0.5 0.4 0.3 0.3 0.3 0.3 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.144 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND 3.3V /VNEG_IN VIN VBUS /+22V /-15V /-20V /15V EPD_VDD --power-nets-widths 0.3 0.3 0.3 0.5 0.4 0.3 0.3 0.3 0.3 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/epdiy_kaleido
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.144 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND 3.3V /VNEG_IN VIN VBUS /+22V /-15V /-20V /15V EPD_VDD --power-nets-widths 0.3 0.3 0.3 0.5 0.4 0.3 0.3 0.3 0.3 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0
