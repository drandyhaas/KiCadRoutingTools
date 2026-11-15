#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/bus_pirate5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/bus_pirate5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/bus_pirate5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/bus_pirate5.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.6548 --via-drill 0.35 --track-width 0.2032 --clearance-ceiling 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/bus_pirate5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_qfn.kicad_pcb --component U103 --nets '*' '!GND' '!+3V3' --clearance 0.15 --width 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/bus_pirate5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_qfn.kicad_pcb --component U103 --nets '*' '!GND' '!+3V3' --clearance 0.15 --width 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/bus_pirate5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/bus_pirate5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_qfn.kicad_pcb step3_capclear.kicad_pcb --clearance 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/bus_pirate5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/bus_pirate5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diff.kicad_pcb 'USB_*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/bus_pirate5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/bus_pirate5
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.2032 --via-size 0.6548 --via-drill 0.35 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +VUSB +1V1 VREG_VIN 'Net-(U103-ADC_AVDD)' 'Net-(J202-VBUS-PadA4B9)' --power-nets-widths 0.4064 0.3048 0.3048 0.254 0.254 0.2032 0.3048 --max-ripup 5 --no-bga-zone
