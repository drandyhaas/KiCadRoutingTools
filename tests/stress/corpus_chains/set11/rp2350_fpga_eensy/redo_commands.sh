#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 qfn_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 bga_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/rp2350_fpga_eensy
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set11/rp2350_fpga_eensy.kicad_pcb step1_planes.kicad_pcb --nets GND GND +3V3 --plane-layers In1.Cu In3.Cu In4.Cu --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.076 --via-size 0.25 --via-drill 0.15 --track-width 0.0762 --hole-to-hole-clearance 0.2 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/rp2350_fpga_eensy
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb step2_fanout_u2.kicad_pcb --component U2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' '!+3V3' --escape-method auto --via-size 0.25 --via-drill 0.15 --track-width 0.0762 --clearance 0.09 --grid-step 0.05 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/rp2350_fpga_eensy
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u2.kicad_pcb --component U2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' '!+3V3' --escape-method auto --via-size 0.25 --via-drill 0.15 --track-width 0.0762 --clearance 0.09 --grid-step 0.05 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/rp2350_fpga_eensy
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u2.kicad_pcb step2b_clearance_u2.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/rp2350_fpga_eensy
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_clearance_u2.kicad_pcb --output step3_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' '!+3V3' --width 0.0762 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --grid-step 0.05 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/rp2350_fpga_eensy
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_fanout_u6.kicad_pcb step3b_clearance_u6.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/rp2350_fpga_eensy
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3b_clearance_u6.kicad_pcb step4_diffpairs.kicad_pcb '/Power Supply and USB/D+' '/Power Supply and USB/D-' /RP2354A/USB_D+ /RP2354A/USB_D- --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --no-gnd-vias --grid-step 0.05 --fab-tier advanced
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/rp2350_fpga_eensy
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.0762 --clearance-ceiling 0.09 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +1V1 /RP2354A/1V1 /VIN 'Net-(U6-VREG_AVDD)' --power-nets-widths 0.4 0.4 0.3 0.3 0.4 0.25 --max-ripup 5 --grid-step 0.05 --fab-tier advanced
